#pragma once
#ifndef _DATA_STRUCTURE_H_
#define _DATA_STRUCTURE_H_
#include <omp.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <boost/algorithm/string.hpp>
#include "KDTree.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

#define NUMBER_OF_THREAD 16

using namespace std;
using namespace Eigen;

struct MinMaxValue {
	double max_value, min_value;
	MinMaxValue() : max_value(0), min_value(0) {

	}

	bool operator == (const MinMaxValue& rhs) {
		return ((this->max_value == rhs.max_value) && (this->min_value == rhs.min_value));
	}
};

struct CorIndex
{
	CorIndex() : p_a_(-1),
		p_b_(-1),
		len(0) { }

	int p_a_;						// Mov piece, pointer 
	int p_b_;						// Fix piece, pointer
	double len;						// length
};

struct CorPair
{
	Vector3d p_A, p_B;
	Vector3d n_A, n_B;
	Vector3d line_n_A, line_n_B;	// Line normal
	int index_A, index_B;			// breakline point data index indicating 'BreakLine' class data index.

	CorPair& operator =(int a)
	{
		this->p_A << a, a, a;
		this->p_B << a, a, a;
		this->n_A << a, a, a;
		this->n_B << a, a, a;
		this->line_n_A << a, a, a;
		this->line_n_B << a, a, a;

		return *this;
	}

	CorPair& operator ()(const Vector3d p_0,
		const Vector3d n_0,
		const Vector3d p_1,
		const Vector3d n_1)
	{
		this->p_A = p_0;
		this->p_B = p_1;
		this->n_A = n_0;
		this->n_B = n_1;

		return *this;
	}

	CorPair& operator =(CorPair& A)
	{
		this->p_A = A.p_A;
		this->p_B = A.p_B;
		this->n_A = A.n_A;
		this->n_B = A.n_B;
		this->line_n_A = A.line_n_A;
		this->line_n_B = A.line_n_B;
		this->index_A = A.index_A;
		this->index_B = A.index_B;

		return *this;
	}

	void SwitchAB(void) {
		Vector3d p_A_pre = this->p_A;
		Vector3d n_A_pre = this->n_A;
		Vector3d line_n_A_pre = this->line_n_A;
		int index_A_pre = this->index_A;

		this->p_A = this->p_B;
		this->n_A = this->n_B;
		this->line_n_A = this->line_n_B;
		this->index_A = this->index_B;
		this->p_B = p_A_pre;
		this->n_B = n_A_pre;
		this->line_n_B = line_n_A_pre;
		this->index_B = index_A_pre;
	}
};

struct Corres
{
	Corres() {
		index_A = 0; 
		index_B = 0;
		cor.clear();
	}

	vector<CorPair> cor;
	int index_A, index_B;			// What is the 'A' shard and 'B' shard, starts 1
};

class BreakLine
{
public:
	BreakLine();
	virtual ~BreakLine();
	BreakLine(const BreakLine& rhs);
	BreakLine(BreakLine&& rhs);
	BreakLine& operator=(const BreakLine& rhs);
	BreakLine& operator=(BreakLine&& rhs);

	void BuildTree(void);
	void RemoveTree(void);
	void ReadAxis(const string& file);
	void ReadPCDFileWithInfo(const string& pcdFilePath);	// Read PCD format file
	void CalculateLineNormal(void);
	void ChangeSequence(MatrixXd& src, int mode);			// Change sequence(only one matrix)
	void ChangeOrder(void);									// Change order of breakLine
	void Remove(void);
	

public:
	MatrixXd			point_;					// 3*n matrix
	MatrixXd			normal_;				// 3*n matrix
	MatrixXd			line_normal_;			// 3*n matrix fractur surface normal
	vector<MatrixXd>	feature_;				// 3*n matrix, 0 : diff-dist, 1 : diff-height, 2 : diff-theta, 3 : dist, 4 : height, 5 : theta, 6 : Thick
	MatrixXd			index_;					// 0 : start, 1 : end, 2 : rim presence // 0 : regular piece, 1 : Rim piece, 2 : Base piece, 3 : Base with rim piece/ minus : fractured base
	vector<Vector3d>	axis_point_;			// Axis starting point
	vector<Vector3d>	axis_norm_;				// Axis normal
	vector<int>			point_index_;			// Point data index
	bool				is_seg_rim_;			// Is this piece have rim?
	bool				is_seg_base_;			// Is this piece base?
	bool				is_sane_base_;			// Is this base perfect or fractured
	bool				is_moving_;				// Is this piece already moved?
	bool				is_inverted_order_;

	kdtree* tree_;
private:
	bool				is_tree_;
};

class Geom
{
public:
	Geom() {
		is_matching_ = false;
		is_thickness_ = false;
	};
	~Geom() {};

	void LoadSurface(const string& sur_in,
		const string& sur_out,
		int num_data = 0);

	void SurMove(const Matrix3d& R,
		const Vector3d& t);

	// Only move m_sBreakLine axis
	void AxisMove(const Matrix3d& R,
		const Vector3d& t);		

	// Move all data include axis
	void Move(const Matrix3d& R,
		const Vector3d& t);			

	// Move data include axis without surface
	void MoveWOSurface(const Matrix3d& R,
		const Vector3d& t);	

	void RemoveData(void);

public:
	BreakLine edge_line_;
	BreakLine sur_in_;
	BreakLine sur_out_;
	BreakLine sur_frac_;
	bool is_matching_;
	bool is_thickness_;
};

class Trans
{
public:
	Trans() {
		Initialize();
	};
	~Trans() {};
	
	void Set(Matrix3d Ri,
		Vector3d ti,
		int index = 0,
		int toward = 0);
	void Set(Matrix4d Ti,
		int index = 0,
		int toward = 0);

	// Accumulate Ri and ti to R and t
	void Input(const Matrix3d Ri,
		Vector3d ti);					
	void Input(const Matrix4d Ti);	

	void Print();

	void Save(const string& filePath);

	void Output(Matrix4d& T_out) const;
	void Output(Matrix3d& R_out,
		Vector3d& t_out) const;

	void InvOut(Matrix4d& T_out) const;
	void InvOut(Matrix3d& R_out,
		Vector3d& t_out) const;

	bool isSimilar(const Trans& comp,
		double rad_threshold = 0.5,
		double t_threshold = 10);
	bool isSimilar(const Matrix3d& R_c,
		const Vector3d& t_c,
		double rad_threshold = 0.5,
		double t_threshold = 10);
	bool isSimilar(const Matrix3d& R,
		double rad_threshold = 0.5);

	void Initialize(void);
	void Read(const string& path);

	Trans operator *(Trans& const rhs) {
		if (this->toward_ == rhs.index_) {
			Trans out;
			Matrix3d R_out = Matrix3d::Identity(), R;
			Vector3d t_out = { 0, 0, 0 }, t;
			rhs.Output(R, t);
			R_out = this->R_ * R;
			t_out = this->R_ * t + this->t_;
			out.Set(R_out, t_out, this->index_, rhs.toward_);
			return out;
		}
		else throw std::runtime_error("Class Trans operator  = : Index and toward are not matching!");
	}

public:
	int index_;			// Indexing itself starts from 1
	int toward_;		// This piece transfort to 'toward' piece

private:
	Matrix3d R_;		// Rotation matrix
	Vector3d t_;		// Translation matrix
	Matrix4d T_;		// Transformation matrix
};

struct CycleNode {
	CycleNode() {
		score = 11;
		inlier = 0;
		c_plausibility = false;
	}

	bool operator <(const CycleNode& a) {
		return this->edges.size() < a.edges.size();
	}

	vector<int> edges;		// LCS output edges number start from 0, 1, 2
	vector<int> nodes;		// Node number start from 1, 2, 3
	double score;			// loop matching score
	int inlier;
	bool c_plausibility;	// cycle compatibility
};

struct LCSpoint
{
	int x, y;

	LCSpoint() : x(0), y(0) {

	}

	LCSpoint operator =(int a) {
		this->x = a;
		this->y = a;

		return *this;
	}

	// L2 norm 
	double operator -(LCSpoint& a) {
		double out(0);
		out = sqrt((this->x - a.x) * (this->x - a.x) + (this->y - a.y) * (this->y - a.y));
		return out;
	}
};

class LCSIndex {
public:

	LCSIndex() : size_(0),
		cluster_(0),
		shard_x_(0),
		shard_y_(0),
		score_(0),
		inliner_(0) 
	{
		start_.x = 0;
		start_.y = 0;
		end_.x = 0;
		end_.y = 0;
		c_plausibility_ = false;
		remove_ = false;
		trans_.index_ = shard_x_;
		trans_.toward_ = shard_y_;
		area_ = 0;
	};
	~LCSIndex() {};

	bool operator ==(const LCSIndex& b) {
		if ((this->shard_x_ == b.shard_x_) && (this->shard_y_ == b.shard_y_)) {
			if (this->size_ == b.size_) {
				if (this->end_.x == b.end_.x && this->end_.y == b.end_.y) {
					return true;
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}

	bool operator <(const LCSIndex& rhs) const {
		return this->cluster_ < rhs.cluster_;
	}

	LCSIndex operator-(int a) {
		LCSIndex temp;
		temp.size_ = this->size_ - a;
		temp.start_.x = this->start_.x - a;
		temp.end_.x = this->end_.x - a;
		temp.start_.y = this->start_.y - a;
		temp.end_.y = this->end_.y - a;

		return temp;
	}

	LCSIndex operator=(int a) {
		this->size_ = a;
		this->end_.x = a;
		this->end_.y = a;
		this->start_.x = a;
		this->start_.y = a;
		this->score_ = a;
		this->inliner_ = a;

		return *this;
	}

	bool SamePart(LCSIndex& b);
	void InvertOrder(int shard, int num);	// WARNING : Recommand after changing order, return it back.

public:
	LCSpoint start_, end_;					// Starting number is '1' not '0'
	int shard_x_, shard_y_;					// shard_y : fix, shard_x : mov(Inverse order)
	int size_;
	int cluster_;
	double score_;							// Edge score after piece wise prunning
	int inliner_;							// number of inlier of this matching pair
	int index_;
	bool c_plausibility_;					// Edge check after cycle compatibility checking
	bool remove_;							// Remove this edges? IF right then true
	bool overlap_;							// Does this mathcing overlap to each other?
	double axis_angle_;						// How much the angle difference between two axises : Unit rad
	int axis_index_x_, axis_index_y_;		// Which axis is used in this matching?
	Trans trans_;							// Transformation matrix shard_x to shard_y
	double area_;
};

//############################## other function ##############################//
// Shape of point, norm are 3*n
bool isSimilarTrans(const Matrix4d& T_a,
	const Matrix4d& T_b,
	const double& rad_threshold,
	const double& t_threshold);

void MatrixMove(MatrixXd& p,
	const MatrixXd& R,
	const Vector3d& t);

void EdgeLineMove(BreakLine& L,
	const Matrix3d& R,
	const Vector3d& t);

void ReadXYZ(const string& path,
	MatrixXd& point,
	MatrixXd& normal,
	int num_data = 0);	

void ReadPCD(const string& path,
	MatrixXd& point,
	MatrixXd& normal,
	int num_data = 0);	

void SaveEdgeLine(const MatrixXd& src,
	const string& path);

void SaveEdgeLine(const vector<Vector3d>& src,
	const string& path);

void SaveEdgeLine(const BreakLine& src,
	const string& path);

#endif // !_DATA_STRUCTURE_H_