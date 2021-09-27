#pragma once
#ifndef _FILTER_H_
#define _FILTER_H_

#include "../class/data_structure.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

using namespace Eigen;
using namespace std;
#define PI 3.14159265
#define Rad_To_Deg 57.2957951

class Filter
{
	static const int kkernelLevel = 10;

public:
	Filter();
	~Filter();

	// circle : Is the data closed? 0 : No, 1 : Yes
	void Gaussian(double sigma,
		MatrixXd const &in,
		MatrixXd &out
		, int circle);	

	void KernelMake(int samples,
		double sigma);

	// From Lanczos differential
	MatrixXd LanczosDiffLow(MatrixXd const& a,
		int order,
		int circle); 

private:
	double sum_kernel_;
	double weight_;
	MatrixXd kernel_;
	MatrixXd diff_;
};

void CalculateFeatureAxisless(Geom& shard,
	int axis_index = 0);

// (r, theta, z) 3*n matrix
MatrixXd ToCylindricalInterpolation(const BreakLine& breakline,
	bool theta_sort,
	bool r_sort);	
void ToCylindricalInterpolation(const BreakLine& breakline,
	vector<Vector3d>& out,
	bool interpol = true);	

MatrixXd GetThickness(Geom& shard);

#endif