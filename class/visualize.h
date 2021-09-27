#pragma once
#ifndef _VISUALIZE_H_
#define _VISUALIZE_H_

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <Eigen/core>
#include <Eigen/dense>
#include "../class/data_structure.h"
#include "../class/feature_matching.h"

class StateManager;

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> pc_xyz;
typedef pcl::PointCloud<pcl::Normal> pc_normal;
typedef pcl::PointCloud<pcl::PointNormal> pc_cloud;
typedef	pc_xyz::Ptr ptr_xyz;
typedef pc_normal::Ptr ptr_norm;
typedef pc_cloud::Ptr ptr_cloud;
typedef pcl::visualization::PCLVisualizer::Ptr view;

class Visualize
{
public:

	Visualize() {}
	~Visualize() {}

	void MakePointCloud(const MatrixXd& data,
		const MatrixXd& norm,
		string name = "Origin");

	void MakeMesh(string& objFilePath,
		string name = "Mesh");

	void MakeMesh(pcl::PolygonMesh& input,
		string name = "Mesh");

	void AddPointCloud(view& viewer,
		int r = 255,
		int g = 255,
		int b = 0,
		int size = 3);

	void AddMesh(view& viewer);

	void AddNormal(view& viewer,
		int level = 1,
		int scale = 1);

	// update the pcl data
	void UpdateData(view& viewer,
		const MatrixXd& data,
		const MatrixXd& norm,
		int r = 255,
		int g = 255,
		int b = 0);

	void UpdateMesh(view& viewer,
		const pcl::PolygonMesh& mesh);

	// Transform the mesh using 'trans' matrix
	void MeshTransform(const Matrix4d& trans,
		view& viewer);		
	void MeshTransform(const Matrix3d& R,
		const Vector3d& t,
		view& viewer);

	void DataTransform(const Matrix4d& trans,
		view& viewer);
	void DataTransform(const Matrix3d& R,
		const Vector3d& t,
		view& viewer);

	void Transform(const Matrix4d& trans,
		view& viewer);
	void Transform(const Matrix3d& R,
		const Vector3d& t,
		view& viewer);

	void RemovePoint(view& viewer);

	void RemoveMesh(view& viewer);

	void TurnOffData(view& viewer);

	void OutData(ptr_xyz& out_data,
		ptr_norm& out_norm);

	void OutMesh(pcl::PolygonMesh& out_mesh);

	void SaveMesh(const string& path) const;

private:
	string				name_;			// Point cloud name
	string				mesh_name_;		// Mesh data name
	string				norm_name_;		// normal data name
	ptr_xyz				data_;			// Point cloud data
	ptr_norm			norm_;			// Point cloud normal
	pc_xyz				mesh_cloud_;	// Point cloud mesh
	pcl::PolygonMesh	mesh_;			// Mesh
	int					r_, g_, b_;
};

class CloudCorres
{
public:
	CloudCorres() { showing_ = false; }
	CloudCorres(ptr_xyz P_A, ptr_xyz P_B) : pc_A_(P_A), pc_B_(P_B) { showing_ = false; }
	~CloudCorres() {}

	void VisualizeCor(string name, view& viewer);
	void Update(view& viewer, ptr_xyz P_A, ptr_xyz P_B);
	void Remove(view& viewer);

	ptr_xyz pc_A_;
	ptr_xyz pc_B_;
	vector<string> name_;
	bool showing_;
};

class VisSwitchVariables {
public:
	bool obj_ = false;			// 'o' key
	bool save_ = false;			// 's' key
	bool first_ = false;		// Space bar
	bool right_ = false;		// '->' key
	bool left_ = false;			// '<-' key

	void KeyEvent(const string& str, const bool keydown);
};

//############################## Other Functions ##############################//
ptr_xyz MatrixtoCloud(const MatrixXd& p);

ptr_norm MatrixtoCloudNorm(const MatrixXd& p);

CloudCorres CorresToCloud(const Corres& cor);	// Change cor.p_A to ptr_xyz

void VisCurrentState(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index,
	vector<Visualize>& pc_overlap,
	const vector<Trans>& T_axis);

void TurnoffandReturn(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index);

void ObjVisualize(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index);

void SaveResult(const vector<Visualize>& pc,
	const StateManager& manager,
	int index,
	const string path);

#endif