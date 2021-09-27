#include "filter.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace Eigen;
using namespace std;

inline double Gauss(double sigma, double x)
{
	double expVal = -1 * (pow(x, 2) / (2*pow(sigma, 2)));
	double divider = sqrt(2 * PI * pow(sigma, 2));
	return (1 / divider) * exp(expVal);
}

// a : signal , b : filter
inline MatrixXd Conv(MatrixXd a, MatrixXd b)
{
	int size = a.rows();
	int sizeb = b.rows();
	int window(0);
	MatrixXd out(size, 1);
	double sum(0);

	for (int i = 0; i < size; i++) {
		if (i >= sizeb) window = sizeb;
		else  window = i;

		for (int j = 0; j < window; j++) {
			sum += b(j, 0) * a(i - j);
		}
		out(i, 0) = sum;
		sum = 0;
	}

	return out;
}

Filter::Filter()
	: sum_kernel_(0.0), weight_(0.0)
{
	kernel_.resize(kkernelLevel, 1);
}

Filter::~Filter()
{}

void Filter::KernelMake(int samples, double sigma)
{
	kernel_.resize(samples, 1);
	bool doubleCenter = false;
	if (kkernelLevel % 2 == 0) {
		doubleCenter = true;
		samples--;
	}

	int steps = (samples - 1) / 2;
	double stepSize = (3 * sigma) / steps;

	for (int i = steps; i >= 1; i--) {
		kernel_(steps - i, 0) = Gauss(sigma, -1*i*stepSize);
		kernel_(kernel_.rows() + i - steps - 1, 0) = Gauss(sigma, i * stepSize);
	}
	kernel_(steps, 0) = Gauss(sigma, 0);
	if (doubleCenter) kernel_(steps + 1, 0) = Gauss(sigma, 0);

	sum_kernel_ = 0;
	for (int i = 0; i < kernel_.rows(); i++) {
		sum_kernel_ += kernel_(i, 0);
	}
	weight_ = 1 / sum_kernel_;
}


MatrixXd Filter::LanczosDiffLow(MatrixXd const& a, int order = 5, int circle = 0)
{
	int n = a.rows();

	if (n < 4) {
		cout << "size of matrix is wrong" << endl;
	}

	diff_.resize(n, 1);

	int m = (order - 1) / 2;
	double temp(0);
	int start(m);

	if (circle) start = 0;

	for (int i = start; i < n - start; i++) {
		temp = 0;
		for (int j = 1; j < m + 1; j++) {
			int front = i + j;
			int back = i - j;

			if (front > n - 1) front = i + j - n;
			if (back < 0) back = i - j + n;
			temp += j * (a(front, 0) - a(back, 0)) / (m * (m + 1) * (2 * m + 1));
		}
		diff_(i, 0) = 3 * temp;
	}

	if(!circle) {
		for (int i = 0; i < m; i++) {
			diff_(i, 0) = a(i + 1) - a(i, 0);
			diff_(n - i - 1, 0) = a(n - i - 1, 0) - a(n - i - 2, 0);
		}
	}

	return diff_;
}

void Filter::Gaussian(double sigma,
	MatrixXd const& in,
	MatrixXd& out,
	int circle)
{
	int samples = kkernelLevel;
	KernelMake(samples, sigma);
	int sampleSide = samples / 2;
	int valueIdx = samples / 2 + 1;
	int ubound = in.rows();
	out.resize(ubound, 1);
	MatrixXd temp(ubound, 1);
	temp = in;
	int start(sampleSide);

	if (circle) start = 0;

	else if (!circle) {
		for (int i = 0; i < sampleSide; i++) {
			out(i, 0) = in(i, 0);
			out(ubound - i - 1) = in(ubound - i - 1);
		}
	}

	for (int i = start; i < ubound - start; i++) {
		double sum = 0;
		int sampleCtr = 0;
		int init = i - sampleSide;
		int limit = i + sampleSide;

		for (int j = init ; j < limit ; j++) {
			int index(j);
			if (j < 0) index = j + ubound;
			else if (j > ubound - 1) index = j - ubound;

			int sampleWeightIndex = sampleSide + (j - i);
			sum += kernel_(sampleWeightIndex, 0) * temp(index);
			sampleCtr++;
		}
		double smoothed = sum * weight_;
		out(i, 0) = smoothed;
	}
}


void CalculateFeatureAxisless(Geom& shard, int axis_index)
{
	Filter filter;

	if (!shard.is_matching_) return;

	// Feature calculation
	int number_of_points = shard.edge_line_.point_.cols();
	MatrixXd Dist(number_of_points, 1), Thickness(number_of_points, 1);
	MatrixXd Height(number_of_points, 1), Theta(number_of_points, 1);

	for (int i = 0; i < number_of_points; i++) {
		Vector3d r = { shard.edge_line_.point_(0, i), shard.edge_line_.point_(1, i), 0 };
		Dist(i) = r.norm();
		Theta(i) = atan2(shard.edge_line_.point_(1, i), shard.edge_line_.point_(0, i));
		Height(i) = shard.edge_line_.point_(2, i);
	}
	for (int i = 1; i < Theta.rows(); i++) {
		if (Theta(i, 0) - Theta(i - 1, 0) > PI) {
			Theta(i, 0) = Theta(i, 0) - 2 * PI;
		}
		else if (Theta(i, 0) - Theta(i - 1, 0) < -PI) {
			Theta(i, 0) = Theta(i, 0) + 2 * PI;
		}
	}

	if ((axis_index == 0) && (!shard.is_thickness_)) {
		Thickness = GetThickness(shard);
	}
	else {
		Thickness = shard.edge_line_.feature_[0].row(6);
	}

	MatrixXd Dist_Diff = filter.LanczosDiffLow(Dist, 7, 0);
	MatrixXd Height_Diff = filter.LanczosDiffLow(Height, 7, 0);
	MatrixXd Theta_Diff = filter.LanczosDiffLow(Theta, 7, 0);

	//Calculate Dist*d(Theta)
	for (int i = 0; i < Theta_Diff.rows(); i++) {
		Theta_Diff(i) = Theta_Diff(i) * Dist(i);
	}

	filter.Gaussian(2, Dist_Diff, Dist_Diff, 1);
	filter.Gaussian(2, Height_Diff, Height_Diff, 1);
	filter.Gaussian(2, Theta_Diff, Theta_Diff, 1);

	int num_features = Dist_Diff.rows();

	shard.edge_line_.feature_[axis_index].resize(7, num_features);
	for (int i = 0; i < num_features; i++) {
		shard.edge_line_.feature_[axis_index].col(i) << Dist_Diff(i), 
			Height_Diff(i), 
			Theta_Diff(i), 
			Dist(i),
			Height(i), 
			Theta(i), 
			Thickness(i);
	}
}

MatrixXd GetThickness(Geom& shard)
{
	int num_breakline = shard.edge_line_.point_.cols();
	int num_o_sur = shard.sur_out_.point_.cols();
	MatrixXd thickness(num_breakline, 1);

	int index_out(0);
	for (size_t i = 0; i < num_breakline; i++) {
		Eigen::Vector4f pt(1, 0, 0, 0), line_pt(0, 0, 0, 0), line_dir(1, 1, 0, 0);
		double point2line_disance = 99999999, dist_Tmp = 0;
		line_pt << shard.edge_line_.point_(0, i), shard.edge_line_.point_(1, i), shard.edge_line_.point_(2, i), 0;
		line_dir << shard.edge_line_.normal_(0, i), shard.edge_line_.normal_(1, i), shard.edge_line_.normal_(2, i), 0;

		for (size_t j = 0; j < num_o_sur; j++) {
			pt << shard.sur_out_.point_(0, j), shard.sur_out_.point_(1, j), shard.sur_out_.point_(2, j), 0;
			dist_Tmp = sqrt(pcl::sqrPointToLineDistance(pt, line_pt, line_dir));
			if (dist_Tmp < point2line_disance)
			{
				point2line_disance = dist_Tmp;
				index_out = j;
			}
		}
		
		Eigen::Vector4f a;
		a << shard.sur_out_.point_(0, index_out), shard.sur_out_.point_(1, index_out), 
			shard.sur_out_.point_(2, index_out), 0;

		if ((point2line_disance > 1.0) || (line_dir.dot(a - line_pt) > 0)) {
			thickness(i) = -1.0;
		}
		else {
			thickness(i) = (shard.sur_out_.point_.col(index_out) - shard.edge_line_.point_.col(i)).norm();
		}
		
	}
	
	shard.is_thickness_ = true;

	return thickness;
}

// (r, theta, z) 3*n matrix
MatrixXd ToCylindricalInterpolation(const BreakLine& breakline,
	bool theta_sort,
	bool r_sort)
{
	MatrixXd b1 = breakline.point_;
	int number_of_points = b1.cols();
	vector<Vector3d> cy;
	double base_theta;
	for (int i = 0; i < number_of_points; i++) {
		Vector3d r = { b1(0, i), b1(1, i), 0 }, tmp;
		tmp(0) = r.norm();
		tmp(1) = atan2(b1(1, i), b1(0, i));
		tmp(2) = b1(2, i);
		cy.push_back(tmp);
	}

	for (int i = 1; i < number_of_points; i++) {
		double gap = abs(cy[i](1) - cy[i - 1](1));
		if ((gap > 0.04) && (gap < 3.14)) {
			int num_interpol = gap / 0.04;
			double r_step = (cy[i](0) - cy[i - 1](0)) / num_interpol;
			double theta_step = (cy[i](1) - cy[i - 1](1)) / num_interpol;
			double z_step = (cy[i](2) - cy[i - 1](2)) / num_interpol;
			for (int j = 0; j < num_interpol; j++) {
				Vector3d tmp;
				tmp(0) = cy[i - 1](0) + r_step * (j + 1);
				tmp(1) = cy[i - 1](1) + theta_step * (j + 1);
				tmp(2) = cy[i - 1](2) + z_step * (j + 1);
				cy.push_back(tmp);
			}
		}
	}

	if (theta_sort) {
		sort(cy.begin(), cy.end(), [](Vector3d a, Vector3d b) -> bool {
			return a(1) < b(1);
		});
	}
	else if (r_sort) {
		sort(cy.begin(), cy.end(), [](Vector3d a, Vector3d b) -> bool {
			return a(0) > b(0);
		});
	}
	number_of_points = cy.size();
	MatrixXd output(3, number_of_points);

	for(int i = 0; i < number_of_points; i++){
		output(0, i) = cy[i](0);
		output(1, i) = cy[i](1);
		output(2, i) = cy[i](2);
	}

	return output;
}

void ToCylindricalInterpolation(const BreakLine& breakline,
	vector<Vector3d>& out,
	bool interpol)
{
	MatrixXd b1 = breakline.point_;
	int number_of_points = b1.cols();
	double base_theta;
	for (int i = 0; i < number_of_points; i++) {
		Vector3d r = { b1(0, i), b1(1, i), 0 }, tmp;
		tmp(0) = r.norm();
		tmp(1) = atan2(b1(1, i), b1(0, i));
		tmp(2) = b1(2, i);
		out.push_back(tmp);
	}

	if (interpol) {
		for (int i = 1; i < number_of_points; i++) {
			double gap = abs(out[i](1) - out[i - 1](1));
			if ((gap > 0.04) && (gap < 3.14)) {
				int num_interpol = gap / 0.04;
				double r_step = (out[i](0) - out[i - 1](0)) / num_interpol;
				double theta_step = (out[i](1) - out[i - 1](1)) / num_interpol;
				double z_step = (out[i](2) - out[i - 1](2)) / num_interpol;
				for (int j = 0; j < num_interpol; j++) {
					Vector3d tmp;
					tmp(0) = out[i - 1](0) + r_step * (j + 1);
					tmp(1) = out[i - 1](1) + theta_step * (j + 1);
					tmp(2) = out[i - 1](2) + z_step * (j + 1);
					out.push_back(tmp);
				}
			}

			double r_gap = abs(out[i](0) - out[i - 1](0));
			if ((r_gap > 0.1) && (gap < 3.14)) {
				int num_interpol = r_gap / 0.1;
				double r_step = (out[i](0) - out[i - 1](0)) / num_interpol;
				double theta_step = (out[i](1) - out[i - 1](1)) / num_interpol;
				double z_step = (out[i](2) - out[i - 1](2)) / num_interpol;
				for (int j = 0; j < num_interpol; j++) {
					Vector3d tmp;
					tmp(0) = out[i - 1](0) + r_step * (j + 1);
					tmp(1) = out[i - 1](1) + theta_step * (j + 1);
					tmp(2) = out[i - 1](2) + z_step * (j + 1);
					out.push_back(tmp);
				}
			}
		}
	}
}

