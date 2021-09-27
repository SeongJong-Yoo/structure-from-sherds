#pragma once
#ifndef _RECONSTRUCTION_H_
#define _RECONSTRUCTION_H_

#include <omp.h>
#include "ceres/cost_function.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "../class/data_structure.h"
#include "../class/feature_matching.h"
#include "../class/axis_estimation.h"
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
class RankingSubgraph;

#define CERES_FUNC_TOL			1.0e-6
#define MINIMUM_NUMBER			7
#define INLIER_THRESHOLD		1.5

typedef pcl::PointCloud<pcl::PointNormal> pc_cloud;
typedef pc_cloud::Ptr ptr_cloud;

//############################## Other function ##############################//
bool isConverge(const Matrix4d& T,
	double rad_threshold,
	double t_threshold);	

double isClose2Eye(const Matrix4d& T);
double isClose2Eye(const Matrix3d& R, const Vector3d& t);

void AxisAlignment(BreakLine& data,
	Matrix3d& R_out,
	Vector3d& t_out,
	int axis_index = 0);

void RejectOutlier(Corres& cor,
	double dist_TH,
	double angle_TH);

void MakeBlock(BreakLine& L,
	BreakLine& output,
	int start,
	int end);

// make_section : For using intervals from LCS out, we need to divide the breakline data..
// A = shard_y, B = shard_x 
void MakeSection(vector<BreakLine>& L,
	BreakLine& p_A,
	BreakLine& p_B, 
	LCSIndex& index);	
void MakeSection(vector<Geom>& shard,
	BreakLine& p_A,
	BreakLine& p_B, 
	LCSIndex& index);

void ChangeFormat(const BreakLine& in, ptr_cloud out);

// A = shard_y, B = shard_x
void MakeCor(Corres& c_in,
	BreakLine& in_A,
	BreakLine& in_B,
	bool onetoone);				

void MakeCorWOBuildTree(Corres& c_in,
	BreakLine& in_A,
	BreakLine& in_B,
	bool onetoone);

void MakeRadiusHeight(double& r,
	double& h,
	vector<Vector3d>& data);

bool isEdgeRemoved(const MatrixXd& table, const vector<LCSIndex>& edges);

void Icp(vector<BreakLine>& L,
	vector<Matrix3d>& R,
	vector<Vector3d>& t,
	vector<LCSIndex>& m_LCS,
	CycleNode& cycle, 
	bool pre_cor = true,
	bool onetoone = true,
	bool onetoone_switching = true,
	bool rim = true);

void IcpIncGraphAxis(vector<Geom>& shard, 
	vector<Matrix3d>& R,
	vector<Vector3d>& t,
	RankingSubgraph& graph,
	vector<RankingSubgraph>& pregraph,
	int& inlier,
	bool rim = true, 
	bool axis = false);

void Registration(vector<BreakLine>& L,
	vector<Matrix3d>& R,
	vector<Vector3d>& t,
	LCSIndex& lcs,
	vector<bool>& true_node);

//############################## Cost function ##############################//
class BiaxialCaoErrorWithFixedAxis {
public:
	BiaxialCaoErrorWithFixedAxis(
		const Eigen::Ref<Eigen::Matrix<double, 3, 1>>& point,
		const Eigen::Ref<Eigen::Matrix<double, 3, 1>>& normal,
		const double weight
	) : point_(point), normal_(normal), weight_(weight) {};

	template <typename T>

	bool operator() (
		const T* const omega,
		const T* const trans,
		T* residual
		) const {

		// Transform surface point
		T transformedPoint[3] { (T)0.0, (T)0.0, (T)0.0 };
		T point[3] { T(point_[0]), T(point_[1]), T(point_[2]) };
		ceres::AngleAxisRotatePoint(omega, point, transformedPoint);
		transformedPoint[0] += trans[0];
		transformedPoint[1] += trans[1];
		transformedPoint[2] += trans[2];

		// Transform normal
		T transformedNormal[3] = { (T)0.0, (T)0.0, (T)0.0 };
		T normal[3] = { T(normal_[0]), T(normal_[1]), T(normal_[2]) };
		ceres::AngleAxisRotatePoint(omega, normal, transformedNormal);

		// Set axis point and normal fixed to the origin and the positive z-vector respectively.
		T axisPoint[3] = { (T)0.0, (T)0.0, (T)0.0 };
		T axisNormal[3] = { (T)0.0, (T)0.0, (T)1.0 };

		// First compute Cao error.
		T T1[3], T2[3], T3[3];

		// Compute T1
		T1[0] = transformedPoint[0] - axisPoint[0];
		T1[1] = transformedPoint[1] - axisPoint[1];
		T1[2] = transformedPoint[2] - axisPoint[2];

		// Compute T2 & T3.
		ceres::CrossProduct(T1, axisNormal, T2);
		// T normal[3]{ T(normal_[0]), T(normal_[1]), T(normal_[2]) };
		ceres::CrossProduct(transformedNormal, axisNormal, T3);

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

		residual[0] = T(weight_)*(c - 1.0 / s * ceres::log(ceres::exp(s * -(dist1 - c)) +
			ceres::exp(s * -(dist2 - c))));

		return true;
	}

private:
	Eigen::Vector3d point_;
	Eigen::Vector3d normal_;
	double weight_;
};

class CostFuncDist {
public:
	CostFuncDist() {}

	CostFuncDist(CorPair one, double d)
		: one_pair_(one), weight_d_(d) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const s_B, const Type* const trans_A, const Type* const trans_B, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		ceres::AngleAxisToRotationMatrix(s_B, dummy);
		Eigen::Matrix<Type, 3, 3> R_B(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, t_B, tempn, temp, line_norm_A, line_norm_B;
		t_A << trans_A[0], trans_A[1], trans_A[2];
		t_B << trans_B[0], trans_B[1], trans_B[2];

		temp = (R_A * one_pair_.p_A + t_A) - (R_B * one_pair_.p_B + t_B);

		residual[0] = (Type)weight_d_ * temp(0);
		residual[1] = (Type)weight_d_ * temp(1);
		residual[2] = (Type)weight_d_ * temp(2);
		return true;
	}

private:
	CorPair one_pair_;
	double weight_d_;
};

class CostFuncFixedDist {
public:
	CostFuncFixedDist() {}

	CostFuncFixedDist(CorPair one, double d)
		: one_pair_(one), weight_d_(d) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const trans_A, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, temp;
		t_A << trans_A[0], trans_A[1], trans_A[2];
		temp = (R_A * one_pair_.p_A + t_A) - one_pair_.p_B;

		residual[0] = (Type)weight_d_ * temp(0);
		residual[1] = (Type)weight_d_ * temp(1);
		residual[2] = (Type)weight_d_ * temp(2);
		return true;
	}

private:
	CorPair one_pair_ ;
	double weight_d_;
};

class CostFuncLineDist {
public:
	CostFuncLineDist() {}

	CostFuncLineDist(CorPair one, double l)
		: one_pair_(one), weight_l_(l) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const s_B, const Type* const trans_A, const Type* const trans_B, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		ceres::AngleAxisToRotationMatrix(s_B, dummy);
		Eigen::Matrix<Type, 3, 3> R_B(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, t_B, tempn, temp, line_norm_A, line_norm_B, norm_A, norm_B;
		t_A << trans_A[0], trans_A[1], trans_A[2];
		t_B << trans_B[0], trans_B[1], trans_B[2];

		temp = (R_A * one_pair_.p_A + t_A) - (R_B * one_pair_.p_B + t_B);

		norm_A = (R_A * one_pair_.n_A);
		norm_B = (R_B * one_pair_.n_B);
		line_norm_A = R_A * one_pair_.line_n_A;
		line_norm_B = R_B * one_pair_.line_n_B;

		Type pl_dist_A = line_norm_A(0) * temp(0) + line_norm_A(1) * temp(1) + line_norm_A(2) * temp(2);
		Type pl_dist_B = line_norm_B(0) * temp(0) + line_norm_B(1) * temp(1) + line_norm_B(2) * temp(2);
		Type sur_dist_A = temp(0) * norm_B(0) + temp(1) * norm_B(1) + temp(2) * norm_B(2);
		Type sur_dist_B = temp(0) * norm_A(0) + temp(1) * norm_A(1) + temp(2) * norm_A(2);


		residual[0] = (Type)weight_l_ * pl_dist_A;
		residual[1] = (Type)weight_l_ * pl_dist_B;
		residual[2] = (Type)weight_l_ * sur_dist_A;
		residual[3] = (Type)weight_l_ * sur_dist_B;
		return true;
	}

private:
	CorPair one_pair_;
	double weight_l_;
};

class CostFuncFixedLineDist {
public:
	CostFuncFixedLineDist() {}

	CostFuncFixedLineDist(CorPair one, double l)
		: one_pair_(one), weight_l_(l) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const trans_A, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, tempn, temp, line_norm_A, norm_A;
		t_A << trans_A[0], trans_A[1], trans_A[2];

		temp = (R_A * one_pair_.p_A + t_A) - one_pair_.p_B;

		norm_A = (R_A * one_pair_.n_A);
		line_norm_A = R_A * one_pair_.line_n_A;

		Type pl_dist_A = (Type)(line_norm_A(0) * temp(0) + line_norm_A(1) * temp(1) + line_norm_A(2) * temp(2));
		Type sur_dist_A = (Type)(temp(0) * norm_A(0) + temp(1) * norm_A(1) + temp(2) * norm_A(2));

		residual[0] = (Type)weight_l_ * pl_dist_A;
		residual[1] = (Type)weight_l_ * sur_dist_A;

		return true;
	}

private:
	CorPair one_pair_;
	double weight_l_;
};

class CostFuncNorm {
public:
	CostFuncNorm() {}

	CostFuncNorm(CorPair one, double n)
		: one_pair_(one), weight_n_(n) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const s_B, const Type* const trans_A, const Type* const trans_B, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		ceres::AngleAxisToRotationMatrix(s_B, dummy);
		Eigen::Matrix<Type, 3, 3> R_B(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, t_B, tempn;
		t_A << trans_A[0], trans_A[1], trans_A[2];
		t_B << trans_B[0], trans_B[1], trans_B[2];

		tempn = (R_A * one_pair_.n_A) - (R_B * one_pair_.n_B);

		for (int j = 0; j < 3; j++) {
			residual[j] = (Type)weight_n_ * tempn(j);
		}
		return true;
	}

private:
	CorPair one_pair_;
	double weight_n_;
};

class CostFuncFixedNorm {
public:
	CostFuncFixedNorm() {}

	CostFuncFixedNorm(CorPair one, double n)
		: one_pair_(one), weight_n_(n) { 	}

	template <typename Type>
	bool operator()(const Type* const s_A, const Type* const trans_A, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s_A, dummy);
		Eigen::Matrix<Type, 3, 3> R_A(dummy);

		Eigen::Matrix<Type, 3, 1> t_A, tempn;
		t_A << trans_A[0], trans_A[1], trans_A[2];

		tempn = (R_A * one_pair_.n_A) - one_pair_.n_B;

		for (int j = 0; j < 3; j++) {
			residual[j] = (Type)weight_n_ * tempn(j);
		}
		return true;
	}

private:
	CorPair one_pair_;
	double weight_n_;
};

class CostFuncRim {
public:
	CostFuncRim() {}

	CostFuncRim(MatrixXd pA, double w_r, double w_h)
		: point_(pA), weight_r_(w_r), weight_h_(w_h) { 	}

	template <typename Type>
	bool operator()(const Type* const s, const Type* const trans, const Type* const radius, const Type* const height, Type* residual) const {
		Type dummy[9];

		ceres::AngleAxisToRotationMatrix(s, dummy);
		Eigen::Matrix<Type, 3, 3> R_i(dummy);

		Eigen::Matrix<Type, 3, 1> t_i;
		t_i << trans[0], trans[1], trans[2];

		Eigen::Matrix<Type, 3, 1> tmp;
		tmp = R_i * point_ + t_i;
		Type r_tmp(0), h_tmp(0);

		r_tmp = sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1));	// sqrt(x^2 + y^2)
		h_tmp = tmp(2);

		Type r = *radius;
		Type h = *height;

		residual[0] = (Type)weight_r_ * (*radius - r_tmp);
		residual[1] = (Type)weight_h_ * (*height - h_tmp);

		return true;
	}

private:
	MatrixXd point_;
	double weight_r_, weight_h_;
};

#endif