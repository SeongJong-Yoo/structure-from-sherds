#pragma once
#ifndef _AXIS_ESTIMATION_H_
#define _AXIS_ESTIMATION_H_

#include "Eigen/Eigen"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// jhh37: dummy load surface 
void LoadSurface(Geom* const geomPtr,
	const std::string& innerSurfacePath,
	const std::string& outerSurfacePath);

void ComputePottmannAxis(const Geom* const geomPtr,
	const std::vector<int>& surfaceIndices,
	Eigen::Ref<Vector3d> axisPoint,
	Eigen::Ref<Vector3d> axisNormal);

void ComputePotSACAxis(Geom* const geomPtr,
	Eigen::Ref<Vector3d> axisPoint,
	Eigen::Ref<Vector3d> axisNormal,
	const unsigned int numIters,
	const unsigned int numThreads,
	const double inlierThreshold,
	const bool USE_BOTH_SURFACES,
	const bool REFINE_AXIS);

void RefineAxis(Geom* const geomPtr,
	Eigen::Ref<Eigen::Vector3d> axisPoint,
	Eigen::Ref<Eigen::Vector3d> axisNormal,
	const unsigned int numIters,
	const unsigned int numThreads,
	const double inlierThreshold,
	const bool USE_BOTH_SURFACES);

void RefineAxis(vector<Geom*> const geom_ptr,
	Eigen::Ref<Eigen::Vector3d> axis_point,
	Eigen::Ref<Eigen::Vector3d> axis_normal,
	const unsigned int num_iters,
	const unsigned int num_threads,
	const double inlier_threshold,
	const bool use_both_surfaces);

// Biaxial Cao error defined in the EH2019 paper (PotSAC).
class BiaxialCaoError {
public:
	BiaxialCaoError(const Eigen::Ref<Eigen::Vector3d>& point,
		const Eigen::Ref<Eigen::Vector3d>& normal)
		: point_(point), normal_(normal) {};

	template <typename T>
	bool operator() (const T* const axisPoint,
		const T* const axisNormal,
		T* residual) const {

		// First compute Cao error.
		T T1[3], T2[3], T3[3];

		// Compute T1
		T1[0] = T(point_[0]) - axisPoint[0];
		T1[1] = T(point_[1]) - axisPoint[1];
		T1[2] = T(point_[2]) - axisPoint[2];

		// Compute T2 & T3.
		ceres::CrossProduct(T1, axisNormal, T2);
		T normal[3]{ T(normal_[0]), T(normal_[1]), T(normal_[2]) };
		ceres::CrossProduct(normal, axisNormal, T3);

		T scale = ceres::sqrt((T2[0] * T2[0] + T2[1] * T2[1] + T2[2] * T2[2])
			/ (T3[0] * T3[0] + T3[1] * T3[1] + T3[2] * T3[2]));

		T res1[3];
		res1[0] = T2[0] - scale * T3[0];
		res1[1] = T2[1] - scale * T3[1];
		res1[2] = T2[2] - scale * T3[2];
		// jhh37: added sqrt (different from PotSAC)
		T dist1 = ceres::sqrt(res1[0] * res1[0] + res1[1] * res1[1] + res1[2] * res1[2]);

		T res2[3];
		res2[0] = T2[0] + scale * T3[0];
		res2[1] = T2[1] + scale * T3[1];
		res2[2] = T2[2] + scale * T3[2];
		// jhh37: added sqrt (different from PotSAC)
		T dist2 = ceres::sqrt(res2[0] * res2[0] + res2[1] * res2[1] + res2[2] * res2[2]);

		T c = std::min(dist1, dist2);
		T s{ T(100.0) };

		residual[0] = c - (T)1.0 / s * ceres::log(ceres::exp(s * -(dist1 - c)) +
			ceres::exp(s * -(dist2 - c)));

		return true;
	}

private:
	Eigen::Vector3d point_;
	Eigen::Vector3d normal_;
};

#endif