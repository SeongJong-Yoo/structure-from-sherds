#include "reconstruction.h"
#include "../class/ranking_system.h"

//############################## other function ##############################//
bool isConverge(const Matrix4d& T, double rad_threshold, double t_threshold)
{
	Matrix3d R;
	Vector3d t, w;
	for (int i = 0; i < 3; i++) {
		R.row(i) << T(i, 0), T(i, 1), T(i, 2);
		t[i] = T(i, 3);
	}

	Matrix3d log_R = R.log();
	w << -log_R(1, 2), log_R(0, 2), -log_R(0, 1);

	double deg = w.norm();	// Radian
	double trans = t.norm();
	if ((deg < rad_threshold) && (trans < t_threshold)) {
		return true;
	}
	else {
		return false;
	}
}

double isClose2Eye(const Matrix4d& T) {
	Matrix4d TmI = T;
	for (int i = 0; i < 4; i++) {
		TmI(i, i) -= 1.0;
	}

	return (TmI.norm());
}

double isClose2Eye(const Matrix3d& R, const Vector3d& t)
{
	Matrix3d RmI = R;
	RmI(0, 0) -= 1.0;
	RmI(1, 1) -= 1.0;
	RmI(2, 2) -= 1.0;

	return (RmI.norm() + t.norm());
}

void AxisAlignment(BreakLine& data,
	Matrix3d& R_out,
	Vector3d& t_out,
	int axis_index)
{
	int numberOfPoints = data.point_.cols();
	if (numberOfPoints < 10) return;

	MatrixXd p_norm(3, numberOfPoints);
	for (int i = 0; i < numberOfPoints; i++) {
		p_norm.col(i) = data.point_.col(i) - data.axis_point_[axis_index];
	}

	Vector3d Axis;
	Axis = data.axis_norm_[axis_index] / data.axis_norm_[axis_index].norm();

	MatrixXd R_t(3, 3);
	MatrixXd R(3, 3);
	Vector3d v3 = Axis;
	Vector3d v2(3);
	Vector3d v1(3);
	Vector3d v1_hash(3);
	v1_hash(0, 0) = 1; v1_hash(1, 0) = 0; v1_hash(2, 0) = 0;

	Vector3d x_vector = v1_hash - ((v1_hash.dot(v3)) * v3);
	v1 = x_vector / x_vector.norm();
	v2 = v3.cross(v1);

	R_t.col(0) = v1;
	R_t.col(1) = v2;
	R_t.col(2) = v3;
	R = R_t.transpose();
	R_out = R;
	t_out = -R_out * data.axis_point_[axis_index];

	MatrixXd Q(3, numberOfPoints);
	Vector3d Q_D;
	Q = R * p_norm;
	Q_D = R * Axis;

	double Q_z_mean = Q.row(2).mean();
	for (int i = 0; i < Q.cols(); i++) {
		data.point_(0, i) = Q(0, i);
		data.point_(1, i) = Q(1, i);
		data.point_(2, i) = Q(2, i) - Q_z_mean;
	}
	t_out[2] -= Q_z_mean;

	data.normal_ = R * data.normal_;
	data.line_normal_ = R * data.line_normal_;

	for (int i = 0; i < data.axis_norm_.size(); i++) {
		data.axis_norm_[i] = R_out * data.axis_norm_[i];
		data.axis_point_[i] = R_out * data.axis_point_[i] + t_out;
	}

	data.axis_norm_[axis_index] = { 0, 0, 1 };
	data.axis_point_[axis_index]= { 0, 0, 0 };
}

void RejectOutlier(Corres& cor, double dist_TH, double angle_TH)
{
	vector<double> distance(cor.cor.size()), Norm(cor.cor.size());
	int num_inliers = 0;

	Corres dummy;
	dummy.index_A = cor.index_A;
	dummy.index_B = cor.index_B;
	for (int i = 0; i < cor.cor.size(); i++) {
		Norm[i] = (cor.cor[i].n_A.dot(cor.cor[i].n_B));
		distance[i] = (cor.cor[i].p_A - cor.cor[i].p_B).norm();

		if ((std::abs(distance[i]) < dist_TH) && (Norm[i]) > angle_TH) {
			dummy.cor.push_back(cor.cor[i]);

			num_inliers++;
		}
	}
	cor.cor.clear();
	cor = dummy;
}

void MakeBlock(BreakLine& L,
	BreakLine& output,
	int start,
	int end)
{
	output.point_ = L.point_.block(0, start - 1, 3, end - start + 1);
	output.normal_ = L.normal_.block(0, start - 1, 3, end - start + 1);
	output.line_normal_ = L.line_normal_.block(0, start - 1, 3, end - start + 1);
	output.axis_point_[0] = L.axis_point_[0];
	output.axis_norm_[0] = L.axis_norm_[0];
}

void MakeSection(vector<BreakLine>& L,
	BreakLine& p_A,
	BreakLine& p_B,
	LCSIndex& index)
{
	int s_A = index.start_.y, s_B = index.start_.x;
	int e_A = index.end_.y, e_B = index.end_.x;

	p_A.point_.resize(3, index.size_);
	p_A.normal_.resize(3, index.size_);
	MakeBlock(L[index.shard_y_ - 1], p_A, s_A, e_A);

	L[index.shard_x_ - 1].ChangeOrder();
	p_B.point_.resize(3, index.size_);
	p_B.normal_.resize(3, index.size_);
	MakeBlock(L[index.shard_x_ - 1], p_B, s_B, e_B);
	L[index.shard_x_ - 1].ChangeOrder();
}

void MakeSection(vector<Geom>& shard,
	BreakLine& p_A,
	BreakLine& p_B,
	LCSIndex& index)
{
	int s_A = index.start_.y, s_B = index.start_.x;
	int e_A = index.end_.y, e_B = index.end_.x;

	p_A.point_.resize(3, index.size_);
	p_A.normal_.resize(3, index.size_);
	MakeBlock(shard[index.shard_y_ - 1].edge_line_, p_A, s_A, e_A);

	shard[index.shard_x_ - 1].edge_line_.ChangeOrder();
	p_B.point_.resize(3, index.size_);
	p_B.normal_.resize(3, index.size_);
	MakeBlock(shard[index.shard_x_ - 1].edge_line_, p_B, s_B, e_B);
	shard[index.shard_x_ - 1].edge_line_.ChangeOrder();
}

void ChangeFormat(const BreakLine& in, ptr_cloud out)
{
	size_t num = in.point_.cols();
	out->points.resize(num);

	for (size_t i = 0; i < num; i++) {
		out->points[i].x = in.point_(0, i);
		out->points[i].y = in.point_(1, i);
		out->points[i].z = in.point_(2, i);
		out->points[i].normal_x = in.normal_(0, i);
		out->points[i].normal_y = in.normal_(1, i);
		out->points[i].normal_z = in.normal_(2, i);
	}
}

void MakeCor(Corres& c_in,
	BreakLine& in_A, 
	BreakLine& in_B,
	bool onetoone)
{
	if (in_A.point_.size() == 1 || in_B.point_.size() == 1) return;

	BreakLine p_A = in_A, p_B = in_B;

	p_A.BuildTree();
	p_B.BuildTree();
	kdres* res = NULL;
	Vector3d pA, pB, nA, nB, line_nA, line_nB;
	// In one to one correspondence case, we don't need to consider about the fix or move points
	// Basically p_A is fixed point and p_B is moving point
	// so correspondence is made by p_A
	for (int i = 0; i < p_B.point_.cols(); i++) {
		res = kd_nearest(p_A.tree_, &p_B.point_(0, i));
		int* ptr_index_A = NULL;
		ptr_index_A = (int*)kd_res_item(res, &pA(0, 0));
		nA = p_A.normal_.col(*ptr_index_A);
		line_nA = p_A.line_normal_.col(*ptr_index_A);
		kd_res_free(res);
		CorPair dummy;

		if (onetoone) {
			res = kd_nearest(p_B.tree_, &pA(0, 0));
			int* ptr_index_B = NULL;
			ptr_index_B = (int*)kd_res_item(res, &pB(0, 0));
			nB = p_B.normal_.col(*ptr_index_B);
			line_nB = p_B.line_normal_.col(*ptr_index_B);
			kd_res_free(res);

			if (pB.isApprox(p_B.point_.col(i))) {
				dummy.p_A = pA;
				dummy.p_B = pB;
				dummy.n_A = nA;
				dummy.n_B = nB;
				dummy.line_n_A = line_nA;
				dummy.line_n_B= line_nB;
				dummy.index_A = *ptr_index_A;
				dummy.index_B = *ptr_index_B;
				c_in.cor.push_back(dummy);
			}
		}
		else {
			dummy.p_A = pA;
			dummy.n_A = nA;
			dummy.line_n_A = line_nA;
			dummy.p_B = p_B.point_.col(i);
			dummy.n_B = p_B.normal_.col(i);
			dummy.line_n_B = p_B.line_normal_.col(i);
			dummy.index_A = *ptr_index_A;
			dummy.index_B = i;
			c_in.cor.push_back(dummy);
		}
	}
	p_A.RemoveTree();
	p_B.RemoveTree();
}

void MakeCorWOBuildTree(Corres& c_in,
	BreakLine& in_A,
	BreakLine& in_B,
	bool onetoone)
{
	if (in_A.point_.size() == 1 || in_B.point_.size() == 1) return;
	kdres* res = NULL;
	Vector3d pA, pB, nA, nB, line_nA, line_nB;
	// In one to one correspondence case, we don't need to consider about the fix or move points
	// Basically p_A is fixed point and p_B is moving point
	// so correspondence is made by p_A
	for (int i = 0; i < in_B.point_.cols(); i++) {
		res = kd_nearest(in_A.tree_, &in_B.point_(0, i));
		int* ptr_index_A = NULL;
		ptr_index_A = (int*)kd_res_item(res, &pA(0, 0));
		nA = in_A.normal_.col(*ptr_index_A);
		line_nA = in_A.line_normal_.col(*ptr_index_A);
		kd_res_free(res);

		CorPair dummy;

		if (onetoone) {
			res = kd_nearest(in_B.tree_, &pA(0, 0));
			int* ptr_index_B = NULL;
			ptr_index_B = (int*)kd_res_item(res, &pB(0, 0));
			nB = in_B.normal_.col(*ptr_index_B);
			line_nB = in_B.line_normal_.col(*ptr_index_B);
			kd_res_free(res);

			if (pB.isApprox(in_B.point_.col(i))) {
				dummy.p_A = pA;
				dummy.p_B = pB;
				dummy.n_A = nA;
				dummy.n_B = nB;
				dummy.line_n_A = line_nA;
				dummy.line_n_B = line_nB;
				dummy.index_A = *ptr_index_A;
				dummy.index_B = *ptr_index_B;
				c_in.cor.push_back(dummy);
			}
		}
		else {
			dummy.p_A = pA;
			dummy.n_A = nA;
			dummy.line_n_A = line_nA;
			dummy.p_B = in_B.point_.col(i);
			dummy.n_B = in_B.normal_.col(i);
			dummy.line_n_B = in_B.line_normal_.col(i);
			dummy.index_A = *ptr_index_A;
			dummy.index_B = i;
			c_in.cor.push_back(dummy);
		}
	}
}

void MakeRadiusHeight(double& r, double& h, vector<Vector3d>& data)
{
	vector<double> radius, height;
	int num = data.size();
	int middle_num = (num + 1) / 2;
	radius.resize(num);
	height.resize(num);

	for (int i = 0; i < num; i++) {
		radius[i] = std::sqrt(data[i][0]* data[i][0] + data[i][1]* data[i][1]);
		height[i] = data[i][2];
	}
	sort(radius.begin(), radius.end());
	sort(height.begin(), height.end());
	r = radius[middle_num];
	h = height[middle_num];
}

bool isEdgeRemoved(const MatrixXd& table, const vector<LCSIndex>& edges)
{
	int num = edges.size();
	for (int i = 0; i < num; i++) {
		int x = edges[i].shard_x_ - 1;
		int y = edges[i].shard_y_ - 1;

		if (x > y) {
			if (!table(y, x)) return true;
		}
		else if (y > x) {
			if (!table(x, y)) return true;
		}
	}

	return false;
}

void Icp(vector<BreakLine>& L,
	vector<Matrix3d>& R,
	vector<Vector3d>& t,
	vector<LCSIndex>& m_LCS,
	CycleNode& cycle,
	bool pre_cor, 
	bool onetoone, 
	bool onetoone_switching,
	bool rim)
{
	int num_shard = L.size();
	int num_node = cycle.nodes.size();

	int num_edge = cycle.edges.size();
	vector<Corres> COR;
	vector<Matrix3d> R_i = R;
	vector<Vector3d> t_i = t;
	MatrixXd Table(num_shard, num_shard);
	vector<bool> true_node(num_shard, false);
	double** trans = new double* [num_shard];	// trans : transfortation
	double** s = new double* [num_shard];		// s : rotaion representer
	double w_d(1.0), w_n(3.0), w_line(0.0), w_a(0.0);

	int counter_unsolved(0);
	int max_iteration = 50, ceres_iteration = 100, max_out_loop = 2;

	COR.resize(num_edge);				// number of correspondence s same as number of edges
	for (int i = 0; i < num_shard; ++i) {
		trans[i] = new double[3];
		s[i] = new double[3];
	}

	// Turn off the Rim constraint if only one piece has rim
	int rim_counter = 0;
	for (int i = 0; i < num_node; i++) {
		int n_index = cycle.nodes[i] - 1;
		if (L[n_index].is_seg_rim_) rim_counter++;
	}
	if (rim_counter < 2) rim = false;

	// Iterate until R_i and t_i are not changed
	bool point_to_line = false;
	bool axis = false;
	for (int out_loop = 0; out_loop < max_out_loop; out_loop++) {
		if (out_loop == 1) {
			w_n = 3.0, w_line = 1.0, w_a = 0.0;
			point_to_line = true;
			axis = false;
		}

		for (int iter = 0; iter < max_iteration; iter++) {
			//########## Clear transformation parameters
			for (int i = 0; i < num_shard; i++) {
				for (int j = 0; j < 3; j++) {
					trans[i][j] = 0;
					s[i][j] = 0;
				}
				for (int j = 0; j < num_shard; j++) {
					Table(i, j) = 0;
				}
				true_node[i] = false;
			}

			//############### Make correspondence ###############// 
			// First time make correspondence in general way and then use one to one conditions
			// Because If you use one to one conditions from beginning, then because of small number of correspondences
			// matching results would not good.
			if (onetoone_switching) {
				onetoone = false;
				if (iter > 1) {
					onetoone = true;
					onetoone_switching = false;
				}
			}

			COR.clear();
			for (int i = 0; i < num_edge; i++) {
				int shard_A = m_LCS[cycle.edges[i]].shard_y_;	// real shard number(i.g. piece 1 is 1)
				int shard_B = m_LCS[cycle.edges[i]].shard_x_;	// shard_x : mov, shard_y : fix
				Corres cor_in;
				cor_in.index_A = shard_A, cor_in.index_B = shard_B;	// make corres index(index the connected nodes)
				BreakLine p_A = L[shard_A - 1], p_B = L[shard_B - 1];

				// if use pre-correspondence then you need to divide the breakline data
				if (pre_cor) {
					MakeSection(L, p_A, p_B, m_LCS[cycle.edges[i]]);
					for (int j = 0; j < p_A.point_.cols(); j++) {
						CorPair dummy;
						dummy.p_A = p_A.point_.col(j);
						dummy.p_B = p_B.point_.col(j);
						dummy.n_A = p_A.normal_.col(j);
						dummy.n_B = p_B.normal_.col(j);
						dummy.line_n_A = p_A.line_normal_.col(j);
						dummy.line_n_B = p_B.line_normal_.col(j);
						cor_in.cor.push_back(dummy);
					}
				}

				else {
					//if not then just use whole breakline data to make correspondence.
					MakeCor(cor_in, p_A, p_B, onetoone);	// If you make one to one correspondecne then set onetoone as true
					RejectOutlier(cor_in, 20, 0.7);
				}

				COR.push_back(cor_in);
			}

			pre_cor = false;		// Pre-correspondences are used at only first time.


			//############### Check correspondence number ###############// 
			bool miss_cor = false;
			//#pragma omp parallel for
			for (int i = 0; i < COR.size(); i++) {
				if (COR[i].cor.size() < MINIMUM_NUMBER) {
					miss_cor = true;
					cycle.score = 11.0;
					break;
				}

				// Fill out edge table 
				int s_A = COR[i].index_A - 1;
				int s_B = COR[i].index_B - 1;
				true_node[s_A] = true;
				true_node[s_B] = true;
				Table(s_A, s_B) = 1;
				Table(s_B, s_A) = 1;
			}
			if (miss_cor)
				break;

			//############### Set nonlinear equation ###############// 
			ceres::Problem problem;
			ceres::LossFunction* loss_dist = new ceres::CauchyLoss(5.0);
			ceres::LossFunction* loss_norm = new ceres::CauchyLoss(2.0);
			
			if (point_to_line) {
				for (int i = 0; i < num_edge; i++) {
					for (int k = 0; k < COR[i].cor.size(); k++) {
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncLineDist, 4, 3, 3, 3, 3>(new CostFuncLineDist(COR[i].cor[k], w_line));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s[COR[i].index_A - 1], s[COR[i].index_B - 1], trans[COR[i].index_A - 1], trans[COR[i].index_B - 1]);

						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncNorm, 3, 3, 3, 3, 3>(new CostFuncNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s[COR[i].index_A - 1], s[COR[i].index_B - 1], trans[COR[i].index_A - 1], trans[COR[i].index_B - 1]);
					}
				}
			}
			else {
				for (int i = 0; i < num_edge; i++) {
					for (int k = 0; k < COR[i].cor.size(); k++) {
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncDist, 3, 3, 3, 3, 3>(new CostFuncDist(COR[i].cor[k], w_d));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s[COR[i].index_A - 1], s[COR[i].index_B - 1], trans[COR[i].index_A - 1], trans[COR[i].index_B - 1]);

						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncNorm, 3, 3, 3, 3, 3>(new CostFuncNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s[COR[i].index_A - 1], s[COR[i].index_B - 1], trans[COR[i].index_A - 1], trans[COR[i].index_B - 1]);
					}
				}
			}

			//######################### Axis consistency ###########################// 
			const int point_interval = 1;
			if (axis) {
				ceres::LossFunction* loss_axis = new ceres::CauchyLoss(1.5);
				for (int i = 0; i < num_shard; i++) {
					if (true_node[i]) {
						int num_surf_points = std::min(L[i].point_.cols(), L[i].point_.cols());
						// total # residuals = 2 * num_residuals_per_surface
						int num_residuals_per_surface = num_surf_points / point_interval;
						int point_id;
						for (int k = 0; k < num_residuals_per_surface; ++k) {
							// add 2 residuals per selected surface point
							point_id = k * point_interval;
							ceres::CostFunction* biaxial_cao_error_in =
								new ceres::AutoDiffCostFunction<BiaxialCaoErrorWithFixedAxis, 1, 3, 3>(
									new BiaxialCaoErrorWithFixedAxis(
										L[i].point_.col(point_id),
										L[i].normal_.col(point_id),
										w_a
									)
									);
							problem.AddResidualBlock(biaxial_cao_error_in, loss_axis, s[i], trans[i]);
							ceres::CostFunction* biaxial_cao_error_out =
								new ceres::AutoDiffCostFunction<BiaxialCaoErrorWithFixedAxis, 1, 3, 3>(
									new BiaxialCaoErrorWithFixedAxis(
										L[i].point_.col(point_id),
										L[i].normal_.col(point_id),
										w_a
									)
									);
							problem.AddResidualBlock(biaxial_cao_error_out, loss_axis, s[i], trans[i]);
						}
					}
				}
			}

			//######################### Rim ###########################// 
			//If rim = true : consider the rim constraint.
			if (rim) {
				ceres::LossFunction* loss_rim = new ceres::CauchyLoss(5.0);
				double w_r(1.5), w_h(1.5);
				double R_rim(0), H_rim(0), num_rim(0);
				vector<Vector3d> rim_data;
				for (int i = 0; i < num_node; i++) {
					int node = cycle.nodes[i] - 1;
					for (int j = 0; j < L[node].index_.cols(); j++) {
						if (L[node].index_(2, j)) {
							int start = L[node].index_(0, j);
							int end = L[node].index_(1, j);
							for (int r_c = start - 1; r_c < end; r_c++) {
								rim_data.push_back(L[node].point_.col(r_c));
							}
						}
					}
				}

				MakeRadiusHeight(R_rim, H_rim, rim_data);

				for (int i = 0; i < num_node; i++) {
					int node = cycle.nodes[i] - 1;
					for (int j = 0; j < L[node].index_.cols(); j++) {
						if (L[node].index_(2, j)) {
							int start = L[node].index_(0, j);
							int end = L[node].index_(1, j);
							for (int r_c = start - 1; r_c < end; r_c++) {
								ceres::CostFunction* cost_function_rim =
									new ceres::AutoDiffCostFunction<CostFuncRim, 2, 3, 3, 1, 1>(new CostFuncRim(L[node].point_.col(r_c), w_r, w_h));
								problem.AddResidualBlock(cost_function_rim, loss_rim, s[node], trans[node], &R_rim, &H_rim);
							}
						}
					}
				}
			}

			ceres::Solver::Options options;
			options.max_num_iterations = ceres_iteration;
			options.minimizer_progress_to_stdout = false;
			options.linear_solver_type = ceres::SPARSE_SCHUR;
			options.function_tolerance = CERES_FUNC_TOL;
			options.num_threads = NUMBER_OF_THREAD;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			//############### Update the transformation matrix to data ###############//
			int count(0);	// For stop condition
			for (int i = 0; i < num_node; i++) {
				double s_dummy[9];
				int node = cycle.nodes[i] - 1;
				ceres::AngleAxisToRotationMatrix(s[node], s_dummy);
				Matrix3d R_dummy(s_dummy);
				R_i[node] = R_dummy;
				t_i[node] = Vector3d(trans[node][0], trans[node][1], trans[node][2]);

				EdgeLineMove(L[node], R_i[node], t_i[node]);
				R[node] = R_i[node] * R[node];
				t[node] = t_i[node] + R_i[node] * t[node];

			}
			for (int i = 0; i < num_edge; i++) {
				int shard_A = m_LCS[cycle.edges[i]].shard_y_ - 1;
				int shard_B = m_LCS[cycle.edges[i]].shard_x_ - 1;
				Matrix4d T1 = Matrix4d::Identity(), T2 = Matrix4d::Identity();
				for (int j = 0; j < 3; j++) {
					T1.row(j) << R_i[shard_A].row(j), t_i[shard_A][j];
					T2.row(j) << R_i[shard_B].row(j), t_i[shard_B][j];
				}
				Matrix4d T = T1.inverse() * T2;
				if (isConverge(T, 0.02, 0.5)) {	
					count++;
					int checkpoint = 0;
				}
			}

			// If all transformation matries are not chaged then stop the iteration
			if ((count == num_edge) || axis) {
				MatchingScore(cycle.score, COR);

				// Inlier should be counted of only one single edge.
				if (num_edge == 1) {
					CountInlier(cycle.inlier, COR, INLIER_THRESHOLD);
					double den(0), input_w(0), volume_weight(1);
					for (int i = 0; i < num_shard; i++) {
						if (true_node[i]) {
							den += 100.0;
							input_w += (double)L[i].point_.cols();
						}
					}
					input_w = input_w / den;
					volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
					cycle.inlier = (int)(volume_weight * (double)cycle.inlier);
				}
				break;
			}
			else {
				cycle.score = 11.0;
				cycle.inlier = 0;
				if (iter == (max_iteration - 1)) {
					for (int ei = 0; ei < COR.size(); ei++) {
						if (COR[ei].cor.size() < MINIMUM_NUMBER) {
							out_loop = max_out_loop;
							break;
						}
					}
					if (out_loop != max_out_loop) {
						MatchingScore(cycle.score, COR);

						if (num_edge == 1) {
							CountInlier(cycle.inlier, COR, INLIER_THRESHOLD);
							if (cycle.inlier < MINIMUM_NUMBER) {
								out_loop = max_out_loop;
								cycle.score = 11.0;
								break;
							}

							double den(0), input_w(0), volume_weight(1);
							for (int i = 0; i < num_shard; i++) {
								if (true_node[i]) {
									den += 100.0;
									input_w += (double)L[i].point_.cols();
								}
							}
							input_w = input_w / den;
							volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
							cycle.inlier = (int)(volume_weight * (double)cycle.inlier);
						}
					}
				}
			}
		}
	}


	R_i.clear();
	t_i.clear();
	for (int i = 0; i < num_shard; i++) {
		delete[] trans[i];
		delete[] s[i];
	}
	delete[] trans;
	delete[] s;
}

void Registration(vector<BreakLine>& L,
	vector<Matrix3d>& R,
	vector<Vector3d>& t,
	LCSIndex& lcs, 
	vector<bool>& true_node)
{
	int num_shard = L.size();
	vector<Corres> COR;
	vector<Matrix3d> R_i = R;
	vector<Vector3d> t_i = t;
	MatrixXd Table(num_shard, num_shard);
	double* trans = new double[3];
	double* s = new double[3];
	double w_d(1.0), w_n(3.0), w_line(0.0), w_a(0.0), w_r(1.0), w_h(1.0);
	bool rim = false, pre_cor = true, onetoone = true;
	int counter_unsolved(0);
	int max_iteration = 50, ceres_iteration = 100, max_out_loop = 3;

	int c_node = lcs.shard_x_;
	int set_node = lcs.shard_y_;
	if (true_node[c_node - 1]) {
		c_node = lcs.shard_y_;
		set_node = lcs.shard_x_;
	}

	// Turn off the Rim constraint if only one piece has rim
	int rim_counter = 0;
	if (L[c_node - 1].is_seg_rim_ && L[set_node - 1].is_seg_rim_)
		rim = true;

	// Iterate until R_i and t_i are not changed
	bool point_to_line = false;
	bool axis = false;
	for (int out_loop = 0; out_loop < max_out_loop; out_loop++) {
		if (out_loop == 1) {
			w_n = 3.0, w_line = 1.0;
			point_to_line = true;
			axis = false;
		}
		else if (out_loop == 2) {
			point_to_line = true;
			w_n = 1.0, w_line = 1.0;
			axis = true;
			w_a = 0.1;
		}

		for (int iter = 0; iter < max_iteration; iter++) {	
			//########## Clear transformation parameters
			for (int i = 0; i < num_shard; i++) {
				for (int j = 0; j < num_shard; j++) {
					Table(i, j) = 0;
				}
			}
			for (int j = 0; j < 3; j++) {
				trans[j] = 0;
				s[j] = 0;
			}
			//############### Make correspondence ###############// 
			// First time make correspondence in general way and then use one to one conditions
			// Because If you use one to one conditions from beginning, then because of small number of correspondences
			// matching results would not good.

			COR.clear();
			int shard_A = lcs.shard_y_;	// real shard number(i.g. piece 1 is 1)
			int shard_B = lcs.shard_x_;	// shard_x : mov, shard_y : fix
			BreakLine p_A = L[shard_A - 1], p_B = L[shard_B - 1];
			bool is_cor_fix_make = false;
			vector<int> cor_making_index;

			// if use pre-correspondence then you need to divide the breakline data
			if (pre_cor) {
				Corres cor_in;
				cor_in.index_A = shard_A, cor_in.index_B = shard_B;	// make corres index(index the connected nodes)
				MakeSection(L, p_A, p_B, lcs);
				for (int j = 0; j < p_A.point_.cols(); j++) {
					CorPair dummy;
					dummy.p_A = p_A.point_.col(j);
					dummy.p_B = p_B.point_.col(j);
					dummy.n_A = p_A.normal_.col(j);
					dummy.n_B = p_B.normal_.col(j);
					dummy.line_n_A = p_A.line_normal_.col(j);
					dummy.line_n_B = p_B.line_normal_.col(j);
					cor_in.cor.push_back(dummy);
				}
				if (cor_in.index_A != c_node) {
					for (int i = 0; i < cor_in.cor.size(); i++) {
						cor_in.cor[i].SwitchAB();
					}
				}
				COR.push_back(cor_in);
			}

			else {
				if (!is_cor_fix_make) {
					for (int i = 0; i < num_shard; i++) {
						if (true_node[i]) {
							L[i].BuildTree();
							cor_making_index.push_back(i);
						}
					}
					is_cor_fix_make = true;
				}
				L[c_node - 1].BuildTree();

				int cor_counter(0);
				int total_size = cor_making_index.size();
				vector<Corres> COR_dummy(total_size);
				vector<bool> cor_index(total_size, false);

				omp_set_num_threads(NUMBER_OF_THREAD);
#pragma omp parallel for
				for (int i = 0; i < total_size; i++) {
					int i_B = cor_making_index[i];
					Corres cor_line;
					MakeCorWOBuildTree(cor_line, L[c_node - 1], L[i_B], onetoone);
					RejectOutlier(cor_line, 20, 0.7);
					cor_line.index_A = c_node;
					cor_line.index_B = i_B + 1;
					COR_dummy[i] = cor_line;
					if (cor_line.cor.size() > MINIMUM_NUMBER - 1) {
						cor_counter++;
						cor_index[i] = true;
					}
				}

				for (int i = 0; i < total_size; i++) {
					if (cor_index[i]) {
						COR.push_back(COR_dummy[i]);
					}
				}
				if (is_cor_fix_make) {
					for (int i = 0; i < num_shard; i++) {
						if (true_node[i]) {
							L[i].RemoveTree();
						}
					}
					is_cor_fix_make = false;
				}
				L[c_node - 1].RemoveTree();
			}
			
			pre_cor = false;		// Pre-correspondences are used at only first time.

			//############### Check correspondence number ###############// 
			for (int i = 0; i < COR.size(); ) {
				if (COR[i].cor.size() < MINIMUM_NUMBER) {
					lcs.score_ = 11.0;
					lcs.inliner_ = 0;
					COR.erase(COR.begin() + i);
				}
				else {
					// Fill out edge table 
					int s_A = COR[i].index_A - 1;
					int s_B = COR[i].index_B - 1;
					Table(s_A, s_B) = 1;
					Table(s_B, s_A) = 1;
					i++;
				}
			}
			if (COR.empty()) {
				lcs.score_ = 11.0;
				lcs.inliner_ = 0;
				break;
			}

			//############### Set nonlinear equation ###############// 
			ceres::Problem problem;
			ceres::LossFunction* loss_dist = new ceres::CauchyLoss(5.0);
			ceres::LossFunction* loss_norm = new ceres::CauchyLoss(2.0);

			if (point_to_line) {
				for (int i = 0; i < COR.size(); i++) {
					for (int k = 0; k < COR[i].cor.size(); k++) {
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncFixedLineDist, 2, 3, 3>(new CostFuncFixedLineDist(COR[i].cor[k], w_line));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s, trans);

						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncFixedNorm, 3, 3, 3>(new CostFuncFixedNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s, trans);
					}
				}
			}
			else {
				for (int i = 0; i < COR.size(); i++) {
					for (int k = 0; k < COR[i].cor.size(); k++) {
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncFixedDist, 3, 3, 3>(new CostFuncFixedDist(COR[i].cor[k], w_d));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s, trans);

						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncFixedNorm, 3, 3, 3>(new CostFuncFixedNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s, trans);
					}
				}
			}

			//######################### Axis consistency ###########################// 
			const int point_interval = 1;
			if (axis) {
				ceres::LossFunction* loss_axis = new ceres::CauchyLoss(1.5);
				int num_surf_points = L[c_node - 1].point_.cols();
				// total # residuals = 2 * num_residuals_per_surface
				int num_residuals_per_surface = num_surf_points / point_interval;
				int point_id;
				for (int k = 0; k < num_residuals_per_surface; ++k) {
					// add 2 residuals per selected surface point
					point_id = k * point_interval;
					ceres::CostFunction* biaxial_cao_error_in =
						new ceres::AutoDiffCostFunction<BiaxialCaoErrorWithFixedAxis, 1, 3, 3>(
							new BiaxialCaoErrorWithFixedAxis(
								L[c_node - 1].point_.col(point_id),
								L[c_node - 1].normal_.col(point_id),
								w_a
								)
							);
					problem.AddResidualBlock(biaxial_cao_error_in, loss_axis, s, trans);
					ceres::CostFunction* biaxial_cao_error_out =
						new ceres::AutoDiffCostFunction<BiaxialCaoErrorWithFixedAxis, 1, 3, 3>(
							new BiaxialCaoErrorWithFixedAxis(
								L[c_node - 1].point_.col(point_id),
								L[c_node - 1].normal_.col(point_id),
								w_a
								)
							);
					problem.AddResidualBlock(biaxial_cao_error_out, loss_axis, s, trans);
				}
			}

			//######################### Rim ###########################// 
			//If rim = true : consider the rim constraint.
			double R_rim(0), H_rim(0), num_rim(0);
			if (rim) {
				ceres::LossFunction* loss_rim = new ceres::CauchyLoss(1.5);
				//if (iter > 3) {
				vector<Vector3d> rim_data;
				for (int i = 0; i < num_shard; i++) {
					if (true_node[i]) {
						for (int j = 0; j < L[i].index_.cols(); j++) {
							if (L[i].index_(2, j)) {
								int start = L[i].index_(0, j);
								int end = L[i].index_(1, j);
								for (int r_c = start - 1; r_c < end; r_c++) {
									rim_data.push_back(L[i].point_.col(r_c));
								}
							}
						}
					}
				}

				MakeRadiusHeight(R_rim, H_rim, rim_data);

				for (int j = 0; j < L[c_node - 1].index_.cols(); j++) {
					if (L[c_node - 1].index_(2, j)) {
						int start = L[c_node - 1].index_(0, j);
						int end = L[c_node - 1].index_(1, j);
						for (int r_c = start - 1; r_c < end; r_c++) {
							ceres::CostFunction* cost_function_rim =
								new ceres::AutoDiffCostFunction<CostFuncRim, 2, 3, 3, 1, 1>(new CostFuncRim(L[c_node - 1].point_.col(r_c), w_r, w_h));
							problem.AddResidualBlock(cost_function_rim, loss_rim, s, trans, &R_rim, &H_rim);
						}
					}
				}
			}

			//############### Solve nonlinear equation ###############//
			if (rim) {
				problem.SetParameterBlockConstant(&R_rim);
				problem.SetParameterBlockConstant(&H_rim);
			}

			ceres::Solver::Options options;
			options.max_num_iterations = ceres_iteration;
			options.minimizer_progress_to_stdout = false;
			options.linear_solver_type = ceres::SPARSE_SCHUR;
			// options.trust_region_strategy_type;
			options.function_tolerance = CERES_FUNC_TOL;
			options.num_threads = NUMBER_OF_THREAD;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			//############### Update the transformation matrix to data ###############//
			int count(0);	// For stop condition
			double s_dummy[9];
			ceres::AngleAxisToRotationMatrix(s, s_dummy);
			Matrix3d R_dummy(s_dummy);
			R_i[c_node - 1] = R_dummy;
			t_i[c_node - 1] = Vector3d(trans[0], trans[1], trans[2]);

			EdgeLineMove(L[c_node - 1], R_i[c_node - 1], t_i[c_node - 1]);
			R[c_node - 1] = R_i[c_node - 1] * R[c_node - 1];
			t[c_node - 1] = t_i[c_node - 1] + R_i[c_node - 1] * t[c_node - 1];

			Matrix4d T = Matrix4d::Identity();
			for (int j = 0; j < 3; j++) {
				T.row(j) << R_i[c_node - 1].row(j), t_i[c_node - 1][j];
			}
			if (isConverge(T, 0.02, 0.5) || axis) {	
				MatchingScore(lcs.score_, COR);
				CountInlier(lcs.inliner_, COR, INLIER_THRESHOLD);
				double den(0), input_w(0), volume_weight(1);
				for (int i = 0; i < num_shard; i++) {
					if (true_node[i]) {
						den += 100.0;
						input_w += (double)L[i].point_.cols();
					}
				}
				input_w = input_w / den;
				volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
				lcs.inliner_ = (int)(volume_weight * (double)lcs.inliner_);
				break;
			}
			else {
				lcs.score_ = 11.0;
				lcs.inliner_ = 0;
				if (iter == max_iteration - 1) {
					for (int ei = 0; ei < COR.size(); ei++) {
						if (COR[ei].cor.size() < MINIMUM_NUMBER) {
							out_loop = max_out_loop;
							break;
						}
					}

					if (out_loop != max_out_loop) {
						MatchingScore(lcs.score_, COR);

						CountInlier(lcs.inliner_, COR, INLIER_THRESHOLD);
						if (lcs.inliner_ < MINIMUM_NUMBER) {
							out_loop = max_out_loop;
							lcs.score_ = 11.0;
							break;
						}

						double den(0), input_w(0), volume_weight(1);
						for (int i = 0; i < num_shard; i++) {
							if (true_node[i]) {
								den += 100.0;
								input_w += (double)L[i].point_.cols();
							}
						}
						input_w = input_w / den;
						volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
						lcs.inliner_ = (int)(volume_weight * (double)lcs.inliner_);
					}

				}
			}
		}
	}

	R_i.clear();
	t_i.clear();
	delete[] trans;
	delete[] s;
}

void IcpIncGraphAxis(
	vector<Geom>& shard, 
	vector<Matrix3d>& R, 
	vector<Vector3d>& t, 
	RankingSubgraph& graph, 
	vector<RankingSubgraph>& pregraph,
	int& inlier, 
	bool rim, 
	bool axis) 
{
	int num_shard = shard.size();
	vector<Corres> COR;
	vector<Matrix3d> R_i = R;
	vector<Vector3d> t_i = t;
	vector<LCSIndex> edges = graph.EdgeOut();
	int c_node = graph.NodeOut();
	MatrixXd Table(num_shard, num_shard);
	double** trans = new double* [num_shard];	// trans : transfortation
	double** s = new double* [num_shard];		// s : rotaion representer
	double w_d(1.0), w_n(1.0), w_line(0);
	double w_r(1.5), w_h(1.5);
	double w_a(1.0);
	bool cor_onetoone = false, point_to_line = false;
	int max_iteration = 200, ceres_iteration = 20;

	int counter_unsolved = 0;

	for (int i = 0; i < num_shard; ++i) {
		trans[i] = new double[3];
		s[i] = new double[3];
	}
	for (int i = 0; i < num_shard; i++) {
		for (int j = 0; j < num_shard; j++) {
			graph.simple_graph_(i, j) = 0;
			Table(i, j) = 0;
		}
	}

	// Turn off the Rim constraint if only one piece has rim
	int rim_counter = 0;
	if (shard[c_node - 1].edge_line_.is_seg_rim_) rim_counter++;
	for (int i = 0; i < num_shard; i++) {
		if ((graph.node_[i]) && (shard[i].edge_line_.is_seg_rim_)) {
			rim_counter++;
		}
	}
	if (rim_counter < 2) rim = false;
	int num_out_loop = 2;
	for (int out_loop = 0; out_loop < num_out_loop; out_loop++)
	{
		if (out_loop == 0) {
			axis = false;
			rim = false;
			cor_onetoone = true;
			point_to_line = true;
			w_d = 0.0;
			w_n = 2.0;
			w_line = 1.0;
			w_r = 1.0;
			w_h = 1.0;
			w_a = 0.0;
		}
		else if (out_loop == 1) {
			axis = true;
			rim = true;
			cor_onetoone = true;
			point_to_line = false;
			w_d = 1.0;
			w_n = 2.0;
			w_line = 0.0;
			w_r = 1.0;
			w_h = 1.0;
			w_a = 0.3;
			max_iteration = 1;
			ceres_iteration = 500;
		}
		// Iterate until R_i and t_i are not changed
		for (int iter = 0; iter < max_iteration; iter++) {
			//############### Clear Parameters ###############// 
			for (int i = 0; i < num_shard; i++) {
				for (int j = 0; j < 3; j++) {
					trans[i][j] = 0;
					s[i][j] = 0;
				}
				R_i[i] = Matrix3d::Identity();
				t_i[i] = Vector3d(0, 0, 0);
				for (int k = 0; k < num_shard; k++) {
					graph.simple_graph_(i, k) = 0;
					Table(i, k) = 0;
				}
			}

			//############### Make correspondence ###############// 
			{
				COR.clear();

				//########## First make KD tree
				for (int i = 0; i < num_shard; i++) {
					if (graph.node_[i]) {
						shard[i].edge_line_.BuildTree();
					}
				}
				//########## Make correspondence pair index
				vector<pair<int, int>> pair_index;
				for (int i = 0; i < num_shard; i++) {
					for (int j = i + 1; j < num_shard; j++) {
						if ((graph.node_[i]) && (graph.node_[j])) {
							pair<int, int> pair_dummy;
							pair_dummy.first = i;
							pair_dummy.second = j;
							pair_index.push_back(pair_dummy);
						}
					}
				}

				int cor_counter(0);
				int total_size = pair_index.size();
				vector<Corres> COR_dummy(total_size);
				vector<bool> cor_index(total_size, false);

				omp_set_num_threads(NUMBER_OF_THREAD);
#pragma omp parallel for 
				for (int i = 0; i < total_size; i++) {
					int i_A = pair_index[i].first, i_B = pair_index[i].second;
					Corres cor_line;
					MakeCorWOBuildTree(cor_line, shard[i_A].edge_line_, shard[i_B].edge_line_, cor_onetoone);
					RejectOutlier(cor_line, 20, 0.7);

					cor_line.index_A = i_A + 1;
					cor_line.index_B = i_B + 1;
					COR_dummy[i] = cor_line;
					// If the number of breakline correspondence is over 6, fill out the table
					if (cor_line.cor.size() > MINIMUM_NUMBER - 1) {
						cor_counter++;
						cor_index[i] = true;

						graph.simple_graph_(i_A, i_B) = 1;
						graph.simple_graph_(i_B, i_A) = 1;
						Table(i_A, i_B) = 1;
						Table(i_B, i_A) = 1;
					}
				}

				for (int i = 0; i < total_size; i++) {
					if (cor_index[i]) {
						COR.push_back(COR_dummy[i]);
					}
				}

				//########## After make correspondence, remove KD tree
				for (int i = 0; i < num_shard; i++) {
					if (graph.node_[i]) {
						shard[i].edge_line_.RemoveTree();
					}
				}
			}
			//############### Set nonlinear equation ###############// 
			ceres::Problem problem;
			ceres::LossFunction* loss_dist = new ceres::CauchyLoss(5.0);
			ceres::LossFunction* loss_norm = new ceres::CauchyLoss(2.0);
			int num_cor_points(0);
			//######################### correspondence distance ###########################// 
			if (point_to_line) {
				for (int i = 0; i < COR.size(); i++) {
					int s_x = COR[i].index_A - 1;
					int s_y = COR[i].index_B - 1;
					for (int k = 0; k < COR[i].cor.size(); k++) {
						num_cor_points++;
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncLineDist, 4, 3, 3, 3, 3>(new CostFuncLineDist(COR[i].cor[k], w_line));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s[s_x], s[s_y], trans[s_x], trans[s_y]);
						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncNorm, 3, 3, 3, 3, 3>(new CostFuncNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s[s_x], s[s_y], trans[s_x], trans[s_y]);
					}
				}
			}
			else if (!point_to_line) {
				for (int i = 0; i < COR.size(); i++) {
					int s_x = COR[i].index_A - 1;
					int s_y = COR[i].index_B - 1;
					for (int k = 0; k < COR[i].cor.size(); k++) {
						num_cor_points++;
						ceres::CostFunction* cost_function_dist =
							new ceres::AutoDiffCostFunction<CostFuncDist, 3, 3, 3, 3, 3>(new CostFuncDist(COR[i].cor[k], w_d));
						problem.AddResidualBlock(cost_function_dist, loss_dist, s[s_x], s[s_y], trans[s_x], trans[s_y]);
						ceres::CostFunction* cost_function_norm =
							new ceres::AutoDiffCostFunction<CostFuncNorm, 3, 3, 3, 3, 3>(new CostFuncNorm(COR[i].cor[k], w_n));
						problem.AddResidualBlock(cost_function_norm, loss_norm, s[s_x], s[s_y], trans[s_x], trans[s_y]);
					}
				}
			}

			//######################### Axis consistency ###########################// 
			const int point_interval = 1;
			if (axis) {
				ceres::LossFunction* loss_axis = new ceres::CauchyLoss(1.5);
				for (int i = 0; i < num_shard; i++) {
					if (graph.node_[i]) {
						// calculate the # residuals per surface
						int num_surf_points = shard[i].edge_line_.point_.cols();
						// total # residuals = 2 * num_residuals_per_surface
						int num_residuals_per_surface = num_surf_points / point_interval;
						int point_id;
						for (int k = 0; k < num_residuals_per_surface; ++k) {
							// add 2 residuals per selected surface point
							point_id = k * point_interval;
							ceres::CostFunction* biaxial_cao_error_in =
								new ceres::AutoDiffCostFunction<BiaxialCaoErrorWithFixedAxis, 1, 3, 3>(
									new BiaxialCaoErrorWithFixedAxis(
										shard[i].edge_line_.point_.col(point_id),
										shard[i].edge_line_.normal_.col(point_id),
										w_a
									)
									);
							problem.AddResidualBlock(biaxial_cao_error_in, loss_axis, s[i], trans[i]);
						}
					}
				}
			}

			//######################### Rim condition ###########################// 
			// If rim = true : consider the rim constraint.
			if (rim) {
				ceres::LossFunction* loss_rim = new ceres::CauchyLoss(5.0);
				double R_rim(0), H_rim(0), num_rim(0);
				vector<Vector3d> rim_data;

				for (int i = 0; i < num_shard; i++) {
					if ((graph.node_[i]) && (shard[i].edge_line_.is_seg_rim_)) {
						for (int j = 0; j < shard[i].edge_line_.index_.cols(); j++) {
							if (shard[i].edge_line_.index_(2, j)) {
								int start = shard[i].edge_line_.index_(0, j);
								int end = shard[i].edge_line_.index_(1, j);
								for (int r_c = start - 1; r_c < end; r_c++) {
									rim_data.push_back(shard[i].edge_line_.point_.col(r_c));
								}
							}
						}
					}
				}
				if (!rim_data.empty()) {
					MakeRadiusHeight(R_rim, H_rim, rim_data);

					for (int i = 0; i < num_shard; i++) {
						if ((graph.node_[i]) && (shard[i].edge_line_.is_seg_rim_)) {
							for (int j = 0; j < shard[i].edge_line_.index_.cols(); j++) {
								if (shard[i].edge_line_.index_(2, j)) {
									int start = shard[i].edge_line_.index_(0, j);
									int end = shard[i].edge_line_.index_(1, j);
									for (int r_c = start - 1; r_c < end; r_c++) {
										ceres::CostFunction* cost_function_rim =
											new ceres::AutoDiffCostFunction<CostFuncRim, 2, 3, 3, 1, 1>(new CostFuncRim(shard[i].edge_line_.point_.col(r_c), w_r, w_h));
										problem.AddResidualBlock(cost_function_rim, loss_rim, s[i], trans[i], &R_rim, &H_rim);
									}
								}
							}
						}
					}
				}
			}
			//############### Solve nonlinear equation ###############//
			if (pregraph.size() != 0) {
				//cout << "Constant ceres parameter " << endl;
				for (int i = 0; i < num_shard; i++) {
					if (pregraph.back().node_[i]) {
						problem.SetParameterBlockConstant(s[i]);
						problem.SetParameterBlockConstant(trans[i]);
					}
				}
			}

			ceres::Solver::Options options;
			options.max_num_iterations = ceres_iteration;
			options.minimizer_progress_to_stdout = false;
			options.linear_solver_type = ceres::SPARSE_SCHUR;
			options.function_tolerance = CERES_FUNC_TOL;
			options.num_threads = NUMBER_OF_THREAD;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			//############### Update the transformation matrix to data ###############//
			for (int i = 0; i < num_shard; i++) {
				double s_dummy[9];
				ceres::AngleAxisToRotationMatrix(s[i], s_dummy);
				Matrix3d R_dummy(s_dummy);
				R_i[i] = R_dummy;
				t_i[i] = Vector3d(trans[i][0], trans[i][1], trans[i][2]);

				if (graph.node_[i]) {
					shard[i].Move(R_i[i], t_i[i]);
					R[i] = R_i[i] * R[i];
					t[i] = t_i[i] + R_i[i] * t[i];
				}
			}
			int num_of_corres = COR.size();
			int count(0);
			for (int i = 0; i < num_of_corres; i++) {
				int shard_A = COR[i].index_A - 1;
				int shard_B = COR[i].index_B - 1;
				Matrix4d T1 = Matrix4d::Identity(), T2 = Matrix4d::Identity();
				for (int j = 0; j < 3; j++) {
					T1.row(j) << R_i[shard_A].row(j), t_i[shard_A][j];
					T2.row(j) << R_i[shard_B].row(j), t_i[shard_B][j];
				}
				Matrix4d T = T1.inverse() * T2;
				if (isConverge(T, 0.02, 0.5)) 
					count++;
			}

			if ((count == num_of_corres) || axis)
			{
				if (!isEdgeRemoved(Table, edges)) {
					if (out_loop == num_out_loop - 1)
					{
						// Fill out matched points
						for (int i = 0; i < COR.size(); i++) {
							int s_A = COR[i].index_A - 1;
							int s_B = COR[i].index_B - 1;
							for (int j = 0; j < COR[i].cor.size(); j++) {
								double dist(0);
								dist = (COR[i].cor[j].p_A - COR[i].cor[j].p_B).norm();
								if (dist < 3.0) {
									graph.matched_index_[s_A][COR[i].cor[j].index_A] = true;
									graph.matched_index_[s_B][COR[i].cor[j].index_B] = true;
								}
							}
						}
					}

					CountInlier(inlier, COR, INLIER_THRESHOLD);

					double den(0), input_w(0), volume_weight(1);

					// ##################### This is penalty of shard's size
					for (int i = 0; i < num_shard; i++) {
						if (graph.node_[i]) {
							den += 100.0;
							input_w += (double)shard[i].edge_line_.point_.cols();
						}
					}
					input_w = input_w / den;
					volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
					inlier = (int)(volume_weight * (double)inlier);
				}
				else {
					inlier = -1;
				}
				break;
			}
			else {
				inlier = -1;
				if (iter == max_iteration - 1) {
					for (int ei = 0; ei < COR.size(); ei++) {
						if (COR[ei].cor.size() < MINIMUM_NUMBER) {
							out_loop = num_out_loop;
							break;
						}
					}

					if (out_loop != num_out_loop) {
						CountInlier(inlier, COR, INLIER_THRESHOLD);

						if (inlier < MINIMUM_NUMBER) {
							out_loop = num_out_loop;
							inlier = -1;
							break;
						}

						double den(0), input_w(0), volume_weight(1);

						// ##################### This is penalty of shard's size
						for (int i = 0; i < num_shard; i++) {
							if (graph.node_[i]) {
								den += 100.0;
								input_w += (double)shard[i].edge_line_.point_.cols();
							}
						}
						input_w = input_w / den;
						volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
						inlier = (int)(volume_weight * (double)inlier);
					}
					else if (out_loop == num_out_loop - 1)
					{
						// Fill out matched points
						for (int i = 0; i < COR.size(); i++) {
							int s_A = COR[i].index_A - 1;
							int s_B = COR[i].index_B - 1;
							for (int j = 0; j < COR[i].cor.size(); j++) {
								double dist(0);
								dist = (COR[i].cor[j].p_A - COR[i].cor[j].p_B).norm();
								if (dist < 3.0) {
									graph.matched_index_[s_A][COR[i].cor[j].index_A] = true;
									graph.matched_index_[s_B][COR[i].cor[j].index_B] = true;
								}
							}
						}
					}
				}
			}
		}
	}

	R_i.clear();
	t_i.clear();
	for (int i = 0; i < num_shard; i++) {
		delete[] trans[i];
		delete[] s[i];
	}
	delete[] trans;
	delete[] s;
}

