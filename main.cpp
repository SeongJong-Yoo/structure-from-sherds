#include <iostream>
#include "glog/logging.h"
#include <time.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
#include "ceres/ceres.h"
#include "class/data_path.h"
#include "class/data_structure.h"
#include "class/Visualize.h"
#include "class/reconstruction.h"
#include "class/feature_matching.h"
#include "class/ranking_system.h"

#define TOP_k 5
#define BRANCH_b 3

using namespace std;
using namespace Eigen;

vector<Geom> shard(SHARD_NUMBER);

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Pot reconstruction"));

VisSwitchVariables vis; 


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*> (nothing);
	std::string key_string = event.getKeySym();
	bool key_down = event.keyDown();
	vis.KeyEvent(key_string, key_down);
}

void IinitPairWiseProfileCheck(vector<Geom>& shard, list<LCSIndex> lcs_out)
{
	list<LCSIndex>::iterator iter = lcs_out.begin();
	//########### Pair-wise profile check
	vector<bool> pair_true_node(SHARD_NUMBER, false);
	for (; iter != lcs_out.end(); ) {
		int index = iter->trans_.index_ - 1;
		int toward = iter->trans_.toward_ - 1;
		if (shard[index].edge_line_.is_seg_base_ || shard[toward].edge_line_.is_seg_base_) {
			iter++;
			continue;
		}
		else 
		{
			Matrix3d R;
			Vector3d t;
			iter->trans_.Output(R, t);
			shard[index].Move(R, t);
			vector<Geom*> geom_ptr;
			geom_ptr.push_back(&shard[index]);
			geom_ptr.push_back(&shard[toward]);
			RefineAxis(geom_ptr,
				shard[toward].edge_line_.axis_point_[0],
				shard[toward].edge_line_.axis_norm_[0],
				100, NUMBER_OF_THREAD, 0.5, true);
			vector<Geom*>().swap(geom_ptr);
			Matrix3d R_a, R_a_i;
			Vector3d t_a, t_a_i;
			AxisAlignment(shard[toward].edge_line_, R_a, t_a);
			shard[index].Move(R_a, t_a);

			vector<Vector3d> profile;
			ToCylindricalInterpolation(shard[index].sur_in_, profile, false);
			ToCylindricalInterpolation(shard[toward].sur_in_, profile, false);
			bool profile_matched = ProfileChecking(profile, 5.0, 10);	// 50 35

			//Count profile curve inlier
			int pc_inlier_A = 1, pc_inlier_B = 1;
			int c_node = toward + 1;
			pair_true_node[index] = true;
			CountPCInlier(pc_inlier_A, pair_true_node, shard, c_node, 5.0, 3.0, false);
			pair_true_node[index] = false;
			pair_true_node[toward] = true;
			c_node = index + 1;
			CountPCInlier(pc_inlier_B, pair_true_node, shard, c_node, 5.0, 3.0, false);
			pair_true_node[toward] = false;
			double weight = (pc_inlier_A + pc_inlier_B - 2) / 2 + 1;
			iter->inliner_ = iter->inliner_ * weight;
			//Finish count profile curve inlier

			R_a_i = R_a.inverse();
			t_a_i = -R_a_i * t_a;
			shard[toward].MoveWOSurface(R_a_i, t_a_i);
			Matrix3d R_i, R_i_total;
			Vector3d t_i, t_i_total;
			iter->trans_.InvOut(R_i, t_i);
			R_i_total = R_i * R_a_i;
			t_i_total = R_i * t_a_i + t_i;

			shard[index].Move(R_i_total, t_i_total);
			shard[toward].edge_line_.axis_norm_[0] = { 0, 0, 1 };
			shard[toward].edge_line_.axis_point_[0] = { 0, 0, 0 };
			if (profile_matched)
				iter++;
			else if (!profile_matched) {
				iter = lcs_out.erase(iter);
			}
		}
	}
}

int main(int argc, char** argv)
{
	//#################### PCL viewer setting ####################//
	double calculation_time(0);
	int s_time(0), e_time(0);

	int step_size = shard.size(); 
	if (argv[1] != NULL) {
		step_size = std::atoi(argv[1]);
	}
	//#################### PCL viewer setting ####################//
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// Register keyboard callback :
	viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)viewer.get());
	 
	cout << "#################### Pottery Data load ####################" << endl;
	//#################### Pottery Data load ####################//
	int max_breakline_points(0);
	for (int i = 0; i < SHARD_NUMBER; i++) {
		shard[i].edge_line_.ReadAxis(axis_path[i]);

		// Consider multi-axis shards at the same time
		if(shard_on_off[i])	{
			shard[i].edge_line_.ReadPCDFileWithInfo(file_path[i]);
			shard[i].edge_line_.CalculateLineNormal();
			int breakline_points = shard[i].edge_line_.point_.cols();
			max_breakline_points = max(max_breakline_points, breakline_points);
			shard[i].LoadSurface(surface_in[i], surface_out[i]);
			shard[i].is_matching_ = true;
		}
	}

	cout << "#################### Save initial state ####################" << endl;
	//#################### Save initial state ####################//
	vector<Visualize> pc_origin(SHARD_NUMBER); 
	for (int i = 0; i < SHARD_NUMBER; i++) {
		if (shard_on_off[i]) {
			std::string pointName = "origin_" + std::to_string(i + 1);
			pc_origin[i].MakePointCloud(shard[i].edge_line_.point_, shard[i].edge_line_.normal_, pointName);
			pointName = "o_Mesh" + std::to_string(i + 1);
			pc_origin[i].MakeMesh(obj_path[i], pointName);
		}
	}

	s_time = clock();

	cout << "#################### Change Axis symmetrix to z axis ####################" << endl;
	//#################### Change Axis symmetrix to z axis ####################//
	vector<Trans> T_axis(SHARD_NUMBER);
	for (int i = 0; i < SHARD_NUMBER; i++) {
		if (shard[i].is_matching_) {
			Matrix3d R_d = Matrix3d::Identity();
			Vector3d t_d = { 0, 0, 0 };
			T_axis[i].Set(R_d, t_d, i + 1, i + 1);

			// Align symmetric axis to z-axis
			AxisAlignment(shard[i].edge_line_, R_d, t_d);	
			shard[i].SurMove(R_d, t_d);

			pc_origin[i].UpdateData(viewer, shard[i].edge_line_.point_, shard[i].edge_line_.normal_);
			pc_origin[i].AddPointCloud(viewer);
			pc_origin[i].MeshTransform(R_d, t_d, viewer);
			T_axis[i].Input(R_d, t_d);		// Save transformation matrix to z-axis

			//########## Align all base fragments to same direction
			if (shard[i].edge_line_.is_seg_base_) {
				cout << "Base direction check, shard : " << i + 1 << "  ";
				double direction(0);
				int base_counter(0);
				Vector3d Z_Axis = { 0, 0, 1 };
				for (int j = 0; j < shard[i].sur_in_.normal_.cols(); j++) {
					double axis_angle = acos(Z_Axis.dot(shard[i].sur_in_.normal_.col(j)));
					if (axis_angle < 0.175 || axis_angle > 3.14159 - 0.175) {
						if (axis_angle < 1.57)
							direction++;
						else
							direction--;
					}
				}
				if (direction > 0) {
					cout << ": Flip";
					shard[i].edge_line_.axis_norm_[0] = -1 * shard[i].edge_line_.axis_norm_[0];
					AxisAlignment(shard[i].edge_line_, R_d, t_d);	
					shard[i].SurMove(R_d, t_d);
					pc_origin[i].MeshTransform(R_d, t_d, viewer);
					T_axis[i].Input(R_d, t_d);
				}
				cout << endl;
			}

			CalculateFeatureAxisless(shard[i]);

			//######## Multi axis
			int num_axis = shard[i].edge_line_.axis_norm_.size();
			if (num_axis > 1) {
				for (int j = 1; j < num_axis; j++) {
					Matrix3d R_a, R_i;
					Vector3d t_a, t_i;
					AxisAlignment(shard[i].edge_line_, R_a, t_a, j);
					CalculateFeatureAxisless(shard[i], j);
					R_i = R_a.inverse();
					t_i = -R_i * t_a;
					shard[i].MoveWOSurface(R_i, t_i);
				}
			}
		}
	}

	//#################### Pottery Surface Data load ####################//
	for (int i = 0; i < SHARD_NUMBER; i++) {
		if (shard[i].is_matching_) {
			int breakline_points = shard[i].edge_line_.point_.cols();
			double suf_weight = (double)breakline_points / (double)max_breakline_points;
			int num_sur_points = (int)(100.0 * suf_weight);
			shard[i].LoadSurface(surface_in[i], surface_out[i], num_sur_points);
			Matrix3d R_d = Matrix3d::Identity();
			Vector3d t_d = { 0, 0, 0 };
			T_axis[i].Output(R_d, t_d);
			shard[i].SurMove(R_d, t_d);
		}
	}

	cout << "#################### Feature matching ####################" << endl;
	////#################### Feature matching ####################//
	list<LCSIndex> LCS_out;
	FeatureComp(shard, LCS_out, 25, MINIMUM_NUMBER, 0);
	cout << "Total number : " << LCS_out.size() << endl;

	cout << "#################### Pairwise pruning ####################" << endl;
	int exclusive_index = 3;	 // rim(index 1) or base(index 2) or fractured base(index 3)
	ExclusivelyPickEdge(LCS_out, shard, exclusive_index);

	PairwisePruning(shard, LCS_out);

	IinitPairWiseProfileCheck(shard, LCS_out);

	list<LCSIndex>::iterator iter = LCS_out.begin();
	cout << "Total number pruned : " << LCS_out.size() << endl;

	int count_move_state(0);

	cout << "#################### Incremental graph building ####################" << endl;
	//////#################### Incremental graph building ####################//
	vector<RankingSubgraph> pre_graph;
	bool on_base_reconstruction = true;
	if (LCS_out.empty())
		on_base_reconstruction = false;
	int base_counter(0);
	////#################### First pahse : Reconstruct only fractured base ####################//
	list<LCSIndex> lcs_base = LCS_out;
	cout << "#################### First pahse : Reconstruct only fractured base ####################" << endl;
	while (on_base_reconstruction) {
		base_counter++;
		cout << "Reconstruct base : " << base_counter << endl;
		RankingManager base_manager(10, 5, shard, LCS_out, step_size, path + "Graph Log", exclusive_index);
		base_manager.BuildStep();
		if (!base_manager.out_graph_.empty()) {
			pre_graph.push_back(base_manager.out_graph_[0]);
		}
		for (int i = 0; i < SHARD_NUMBER; i++) {
			if (base_manager.out_graph_[0].node_[i]) {
				if (shard[i].edge_line_.is_seg_base_)
					shard[i].edge_line_.is_seg_base_ = false;
				else
					cout << "Error in a base reconstruction process" << endl;
			}
		}
		LCS_out.clear();
		LCS_out = lcs_base;
		ExclusivelyPickEdge(LCS_out, shard, exclusive_index);
		if (LCS_out.empty())
			on_base_reconstruction = false;
	}

	////#################### Second pahse : Reconstruct all fragments ####################//
	cout << "#################### Second pahse : Reconstruct all fragments ####################" << endl;
	StateManager manager(step_size, path + "Graph Log");
	manager.InitializeWithGraph(TOP_k, BRANCH_b, shard, pre_graph);
	manager.BuildStep();

	int num_total = manager.out_state_.size();

	vector<Visualize> pc_overlap(num_total);

	e_time = clock();

	cout << "Time after data load : " << (e_time - s_time)/1000 << "sec" << endl;

	bool is_whole_shards_used = false;
	count_move_state = 0;

	iter = LCS_out.begin();
	bool is_first_state = false;
	while (!viewer->wasStopped()) {
		viewer->spinOnce(5);
		if (vis.first_) {
			for (int i = 0; i < SHARD_NUMBER; i++) {
				pc_origin[i].TurnOffData(viewer);
			}
			if (count_move_state != 0) {
				TurnoffandReturn(viewer, pc_origin, manager, count_move_state);
				count_move_state = 0;
				cout << "Go to first result" << endl;
			}
			if (count_move_state == 0 && !is_first_state) {
				VisCurrentState(viewer, pc_origin, manager, count_move_state, pc_overlap, T_axis);
				pc_overlap[count_move_state].AddPointCloud(viewer, 255, 0, 0);
				is_first_state = true;
			}
			vis.first_ = false;
		}

		else if (vis.right_) {
			is_first_state = false;
			if (count_move_state >= num_total - 1) {
				cout << "There is no more result. Please press 'spacebar' to go first result" << endl;
			}
			else {
				TurnoffandReturn(viewer, pc_origin, manager, count_move_state);

				pc_overlap[count_move_state].AddPointCloud(viewer);
				count_move_state++;
				VisCurrentState(viewer, pc_origin, manager, count_move_state, pc_overlap, T_axis);

				pc_overlap[count_move_state].AddPointCloud(viewer, 255, 0, 0);
			}
			vis.right_ = false;
		}

		else if (vis.left_) {
			if (count_move_state < 1) {
				cout << "This is first result" << endl;
				is_first_state = true;
			}
			else {
				TurnoffandReturn(viewer, pc_origin, manager, count_move_state);

				pc_overlap[count_move_state].AddPointCloud(viewer);
				count_move_state--;
				VisCurrentState(viewer, pc_origin, manager, count_move_state, pc_overlap, T_axis);

				pc_overlap[count_move_state].AddPointCloud(viewer, 255, 0, 0);
			}
			vis.left_ = false;
		}

		else if (vis.obj_) {
			ObjVisualize(viewer, pc_origin, manager, count_move_state);
			vis.obj_ = false;
		}

		else if (vis.save_) {
			string path_result = path + "Result/";
			SaveResult(pc_origin, manager, count_move_state, path_result);
			vis.save_ = false;
			cout << "#################### Result save finish : " << path_result << endl;
		}
	}
	return 0;
}


