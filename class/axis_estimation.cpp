#include "data_structure.h"
#include "axis_estimation.h"

#include "Eigen/Eigen"
#include "ceres/ceres.h"

#include <thread>
#include <random>
#include <unordered_set>

// convert euclidean to Plucker coordinates
// If n is the normal vector and c is the point vector, then its Plucker notation is
// [n; cross(c,n)] in MATLAB.
inline void Euc2Plucker(const Eigen::Ref<const Eigen::Vector3d>& euc_normal,
	const Eigen::Ref<const Eigen::Vector3d>& euc_point,
	Eigen::Ref<Eigen::Vector3d> plucker_point) {
	plucker_point = euc_point.cross(euc_normal);
}

// N-choose-k by Robert Floyd
// https://stackoverflow.com/questions/28287138/c-randomly-sample-k-numbers-from-range-0n-1-n-k-without-replacement
std::vector<int> pick(int N, int k, std::mt19937& gen)
{
	std::unordered_set<int> elems;
	for (int r = N - k; r < N; ++r) {
		int v = std::uniform_int_distribution<>(1, r)(gen);

		// there are two cases.
		// v is not in candidates ==> add it
		// v is in candidates ==> well, r is definitely not, because
		// this is the first iteration in the loop that we could've
		// picked something that big.

		if (!elems.insert(v).second) {
			elems.insert(r);
		}
	}
	std::vector<int> result(elems.begin(), elems.end());
	// no need to shuffle
	// std::shuffle(result.begin(), result.end(), gen);
	return result;
}

// dummy load surface function
void LoadSurface(Geom* const geom_ptr,
	const std::string& inner_surface_path,
	const std::string& outer_surface_path) {
	ReadXYZ(inner_surface_path, geom_ptr->sur_in_.point_, geom_ptr->sur_in_.normal_);
	ReadXYZ(outer_surface_path, geom_ptr->sur_out_.point_, geom_ptr->sur_out_.normal_);

	// normalize normals
	for (int i = 0; i < geom_ptr->sur_in_.normal_.cols(); ++i)
		geom_ptr->sur_in_.normal_.col(i).normalize();
	for (int i = 0; i < geom_ptr->sur_out_.normal_.cols(); ++i)
		geom_ptr->sur_out_.normal_.col(i).normalize();
	//std::cout << geom_ptr->sur_out_.normal.col(0) << std::endl;
}

// jhh37
void ComputePottmannAxis(const Geom* const geom_ptr,
	const std::vector<int>& surface_indices,
	Eigen::Ref<Vector3d> axis_point,
	Eigen::Ref<Vector3d> axis_normal) {
	// check no. of indices >= 6
	assert(surface_indices.size() > 2);

	size_t num_points{ surface_indices.size() };
	Eigen::Matrix3Xd jacobian_normal_transposed(3, num_points);
	Eigen::Matrix3Xd jacobian_point_transposed(3, num_points);

	// Compute Jacobians
	int num_inner_surface_points = geom_ptr->sur_in_.point_.cols();
	for (int i = 0; i < num_points; ++i) {
		if (surface_indices[i] < num_inner_surface_points) {
			Euc2Plucker(geom_ptr->sur_in_.normal_.col(surface_indices[i]),
				geom_ptr->sur_in_.point_.col(surface_indices[i]),
				jacobian_normal_transposed.col(i));
			jacobian_point_transposed.col(i) = geom_ptr->sur_in_.normal_.col(surface_indices[i]);

		}
		else {
			Euc2Plucker(geom_ptr->sur_out_.normal_.col(surface_indices[i] - num_inner_surface_points),
				geom_ptr->sur_out_.point_.col(surface_indices[i] - num_inner_surface_points),
				jacobian_normal_transposed.col(i));
			jacobian_point_transposed.col(i) = geom_ptr->sur_out_.normal_.col(surface_indices[i] - num_inner_surface_points);
		}
	}

	// Compute JTJ
	Eigen::Matrix3d JTJ11, invJTJ22, JTJ12, JTJ;
	JTJ11 = jacobian_normal_transposed * jacobian_normal_transposed.transpose();
	invJTJ22 = (jacobian_point_transposed * jacobian_point_transposed.transpose()).inverse();
	JTJ12 = jacobian_normal_transposed * jacobian_point_transposed.transpose();

	JTJ = JTJ11 - JTJ12 * invJTJ22 * JTJ12.transpose();

	// Compute SVD and take the last row as the normal.
	Eigen::JacobiSVD<Matrix3d> svd(JTJ, Eigen::DecompositionOptions::ComputeFullV);
	axis_normal = svd.matrixV().col(2);

	// Get axis offset
	axis_point = -axis_normal.cross(invJTJ22 * (JTJ12.transpose() * axis_normal));
}

// jhh37
void ComputePotSACAxis(Geom* const geom_ptr,
	Eigen::Ref<Vector3d> axis_point,
	Eigen::Ref<Vector3d> axis_normal,
	const unsigned int num_iters,
	const unsigned int num_threads,
	const double inlier_threshold,
	const bool use_both_surfaces,
	const bool refine_axis) {
	assert(num_iters > 0);
	assert(num_threads > 0 && num_threads > std::thread::hardware_concurrency());

	// Get # points.
	size_t num_points;
	num_points = geom_ptr->sur_in_.point_.cols();
	std::vector<BiaxialCaoError*> residuals;

	// Add residuals
	for (int i = 0; i < num_points; ++i) {
		residuals.push_back(new BiaxialCaoError(geom_ptr->sur_in_.point_.col(i),
			geom_ptr->sur_in_.normal_.col(i)));
	}
	if (use_both_surfaces) {
		// Add residuals for the outer surface
		size_t num_outer_points = geom_ptr->sur_out_.point_.cols();
		for (int i = 0; i < num_outer_points; ++i) {
			residuals.push_back(new BiaxialCaoError(geom_ptr->sur_out_.point_.col(i),
				geom_ptr->sur_out_.normal_.col(i)));
		}
		num_points += num_outer_points;
	}

	/* random sampling & batch model evaluation */
	int min_num_samples = 6;
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::vector< std::vector<int> > sample_indices(num_iters);
	std::vector<double> costs(num_iters);
	// Multithreading
	std::vector<std::thread> threads(num_threads);
	for (unsigned int t = 0; t < num_threads; ++t) {
		unsigned int lb = t * num_iters / num_threads;
		unsigned int ub = ((t + 1) == num_threads) ?
			num_iters : (t + 1) * num_iters / num_threads;
		threads[t] = std::thread(std::bind([&](const unsigned int start_iter,
			const unsigned int end_iter) {

			Eigen::Vector3d iter_axis_point;
			Eigen::Vector3d iter_axis_normal;
			double iter_residual;
			for (unsigned int iter = start_iter; iter < end_iter; ++iter) {
				// Shuffle indices
				sample_indices[iter] = pick(num_points, min_num_samples, gen);

				// Compute axis
				ComputePottmannAxis(geom_ptr,
					sample_indices[iter],
					iter_axis_point,
					iter_axis_normal);
				costs[iter] = 0.0;
				// Compute cost
				for (int i = 0; i < num_points; ++i) {
					residuals[i]->operator()(iter_axis_point.data(),
						iter_axis_normal.data(),
						&iter_residual);
					// Outliers cannot go beyond the inlier threshold.
					iter_residual = std::min(iter_residual, inlier_threshold);
					costs[iter] += (iter_residual * iter_residual);
				}
			}
		}, lb, ub));
	}
	// Start each thread.
	std::for_each(threads.begin(), threads.end(), [](std::thread& t) { t.join(); });

	// find winning index.
	auto result = std::min_element(costs.begin(), costs.end());
	unsigned int winning_index = std::distance(costs.begin(), result);

	// get final model.
	ComputePottmannAxis(geom_ptr,
		sample_indices[winning_index],
		axis_point,
		axis_normal);

	// refine axis if required
	if (refine_axis) {
		RefineAxis(geom_ptr, axis_point, axis_normal,
			300, num_threads, 1.0, true);
	}
}

// jhh37
void RefineAxis(Geom* const geom_ptr,
	Eigen::Ref<Eigen::Vector3d> axis_point,
	Eigen::Ref<Eigen::Vector3d> axis_normal,
	const unsigned int num_iters,
	const unsigned int num_threads,
	const double inlier_threshold,
	const bool use_both_surfaces) {
	assert(num_iters > 0);
	assert(num_threads > 0 && num_threads > std::thread::hardware_concurrency());

	// instantiate Ceres problem
	ceres::Problem problem;
	ceres::LossFunction* loss_ptr = new ceres::CauchyLoss(inlier_threshold);
	ceres::LocalParameterization* unit_sphere = new ceres::HomogeneousVectorParameterization(3);

	// Add residuals
	size_t num_points = geom_ptr->sur_in_.point_.cols();
	std::vector<BiaxialCaoError*> residuals;
	for (int i = 0; i < num_points; ++i) {
		residuals.push_back(new BiaxialCaoError(geom_ptr->sur_in_.point_.col(i),
			geom_ptr->sur_in_.normal_.col(i)));
		// Create Ceres residual
		ceres::CostFunction* cost_ptr =
			new ceres::AutoDiffCostFunction<BiaxialCaoError, 1, 3, 3>(residuals[i]);
		problem.AddResidualBlock(cost_ptr, loss_ptr, axis_point.data(), axis_normal.data());
	}
	if (use_both_surfaces) {
		// Add residuals for the outer surface
		size_t num_outer_points = geom_ptr->sur_out_.point_.cols();
		for (int i = 0; i < num_points; ++i) {
			residuals.push_back(new BiaxialCaoError(geom_ptr->sur_out_.point_.col(i),
				geom_ptr->sur_out_.normal_.col(i)));
			ceres::CostFunction* cost_ptr =
				new ceres::AutoDiffCostFunction<BiaxialCaoError, 1, 3, 3>(residuals[num_points + i]);
			problem.AddResidualBlock(cost_ptr, loss_ptr, axis_point.data(), axis_normal.data());
		}
		num_points += num_outer_points;
	}

	// Set unit vector manifold parameterization
	problem.SetParameterization(axis_normal.data(), unit_sphere);

	// std::cout << axis_normal << std::endl << axis_point << std::endl;


	// Run the solver!
	const bool display_output = false;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = display_output;
	options.max_num_iterations = num_iters;
	options.function_tolerance = 1.0e-9;
	options.num_threads = num_threads;
	options.use_inner_iterations = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// Make point orthogonal to direction
	axis_point -= axis_point.dot(axis_normal) * axis_normal;
}

void RefineAxis(vector<Geom*> const geom_ptr,
	Eigen::Ref<Eigen::Vector3d> axis_point,
	Eigen::Ref<Eigen::Vector3d> axis_normal,
	const unsigned int num_iters,
	const unsigned int num_threads,
	const double inlier_threshold,
	const bool use_both_surfaces) {
	//assert(num_iters > 0);
	//assert(num_threads > 0 && num_threads > std::thread::hardware_concurrency());

	// instantiate Ceres problem
	ceres::Problem problem;
	ceres::LossFunction* loss_ptr = new ceres::CauchyLoss(inlier_threshold);
	ceres::LocalParameterization* unit_sphere = new ceres::HomogeneousVectorParameterization(3);

	// Add residuals
	size_t num_shard = geom_ptr.size();
	for (size_t j = 0; j < num_shard; j++) {
		size_t num_points = geom_ptr[j]->sur_in_.point_.cols();
		std::vector<BiaxialCaoError*> residuals;
		for (int i = 0; i < num_points; i ++) {
			residuals.push_back(new BiaxialCaoError(geom_ptr[j]->sur_in_.point_.col(i),
				geom_ptr[j]->sur_in_.normal_.col(i)));
			// Create Ceres residual
			ceres::CostFunction* cost_ptr =
				new ceres::AutoDiffCostFunction<BiaxialCaoError, 1, 3, 3>(residuals.back());
			problem.AddResidualBlock(cost_ptr, loss_ptr, axis_point.data(), axis_normal.data());
		}
		if (use_both_surfaces) {
			// Add residuals for the outer surface
			size_t num_outer_points = geom_ptr[j]->sur_out_.point_.cols();
			for (int i = 0; i < num_outer_points; i ++) {
				residuals.push_back(new BiaxialCaoError(geom_ptr[j]->sur_out_.point_.col(i),
					geom_ptr[j]->sur_out_.normal_.col(i)));
				ceres::CostFunction* cost_ptr =
					//new ceres::AutoDiffCostFunction<BiaxialCaoError, 1, 3, 3>(residuals[num_points + i]);
					new ceres::AutoDiffCostFunction<BiaxialCaoError, 1, 3, 3>(residuals.back());
				problem.AddResidualBlock(cost_ptr, loss_ptr, axis_point.data(), axis_normal.data());
			}
			//num_points += num_outer_points;
		}
	}

	// Set unit vector manifold parameterization
	problem.SetParameterization(axis_normal.data(), unit_sphere);

	// std::cout << axis_normal << std::endl << axis_point << std::endl;

	// Run the solver!
	const bool display_output = false;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = display_output;
	options.max_num_iterations = num_iters;
	options.function_tolerance = 1.0e-6;
	options.num_threads = num_threads;
	options.use_inner_iterations = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// Make point orthogonal to direction
	axis_point -= axis_point.dot(axis_normal) * axis_normal;
}