#pragma once
#ifndef _RANKING_SYSTEM_H_
#define _RANKING_SYSTEM_H_

#include <memory.h>
#include <omp.h>
#include <ctime>
#include "../class/data_structure.h"
#include "../class/feature_matching.h"
#include "../class/reconstruction.h"
#include "../class/filter.h"
#include "../class/visualize.h"
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

struct Chunk {
	int node;
	vector<int> i_edge;			// edge index about LCS list
	int inlier;					// number of inlier of this chunk 

	bool operator ==(const Chunk& a) {
		if (this->node != a.node) return false;
		else if (this->inlier != a.inlier) return false;
		else if (this->i_edge.size() != a.i_edge.size()) return false;
		else {
			int count = 0;
			int size = this->i_edge.size();
			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (a.i_edge[i] == this->i_edge[j]) {
						count++;
						break;
					}
				}
			}
			if (count != size) return false;
		}
		return true;
	}
};

class RankingSubgraph {
public:
	RankingSubgraph(list<LCSIndex>& lcs_out, int num_of_shard)
		: lcs_reference_(lcs_out),
		knum_of_shard_(num_of_shard),
		sub_graph_(num_of_shard),
		priority_index_(0) 
	{
		graph_score_ = 0;
		pc_score_ = 0.0;
		root_node_ = -1;
		max_overlap_area_ = 0;
		node_.resize(num_of_shard, false);
		simple_graph_ = MatrixXd::Zero(num_of_shard, num_of_shard);

		Matrix3d I = Matrix3d::Identity();
		Vector3d zero = { 0, 0, 0 };
		T_.resize(num_of_shard);
		for (int i = 0; i < num_of_shard; ++i) {
			T_[i].Set(I, zero, i + 1, i + 1);
		}
		int count(0);
		for (auto iter = lcs_out.begin(); iter != lcs_out.end(); iter++) {
			sub_graph_[count] = *iter;
			count++;
		}
	};

	~RankingSubgraph() {};

	bool operator > (const RankingSubgraph& rhs) const {
		return (this->graph_score_ > rhs.graph_score_);
	}	
	void Copy(RankingSubgraph& input);

	void InputTransLog(int index,
		Matrix3d& R,
		Vector3d& t);

	// Initialize and make T_log matrix
	void MakeTransLog(void);

	vector<string> StringOutTransLog(int index, const vector<Trans>& T_axis);

	bool isSimilarGraph(RankingSubgraph& G_i);

	int NumTrueNode(void);

	// Output current node.
	int NodeOut(void);	

	// Output current connection edge
	vector<LCSIndex> EdgeOut(void);				
	
	void MakePriorityList(const vector<bool>& true_node);

	// Replace lcs_reference to lcs_in
	void ReplaceLCS(list<LCSIndex>& lcs_in);						

private:
	void CombineChunk(list<Chunk>& priority,
		vector<LCSIndex>& lcs);

	// Pick subgraph edges 
	vector<LCSIndex> PickEdges(const vector<bool>& true_node);		

	void PickEdges(const vector<bool>& true_node,
		vector<LCSIndex>& inside_lcs,
		vector<LCSIndex>& outside_lcs);

public:
	int knum_of_shard_;							// Number of shards
	vector<Trans> T_;							// Transformation matrix of all shards.
	list<LCSIndex> lcs_reference_;				// Reference graph which is G in the algorithm. This reference graph varies with each step, which is line 7 in the algorithm 
	int priority_index_;						// Priority index : Which effective chunck is processing now
	vector<Chunk> priority_list_;				// Include processing node and connected edge to true_graph. 	
	vector<LCSIndex> sub_graph_;				// subgraph only includes nodes from (V - V')
	double max_overlap_area_;					// Max overlapped area of this step
	list<LCSIndex> edge_;
	vector<pair<int, int>> edge_log_;			// edge log 
	vector<int>  edge_size_;					// Record of edge size of each step
	vector<MatrixXd> T_log_;					// Transformation log
	vector<int>  node_log_;						// node log
	vector<bool> node_;
	int graph_score_;
	double pc_score_;							// Profile curve score
	int root_node_;								// Root node starts 1
	MatrixXd simple_graph_;						// Simple graph expression, which doesn't include edge region information
	vector<vector<bool>> matched_index_;		// e.g. if shard[0].point_.col(3) is matched then, matched_index_[0][3] is 'true' 
};

struct Step {
	vector<RankingSubgraph> top_rank_graph_;	// Subgraph set which size is preserved as N 
	vector<RankingSubgraph> ext_graph_;			// Extended graph which maximum size is s*N
};

class RankingManager {
public:
	RankingManager() : N_(0), s_(0), step_counter_(0) {};
	RankingManager(int N, 
		int s,
		vector<Geom>& shard, 
		list<LCSIndex>& LCS_out,
		int step_size,
		string& log_path, 
		int exclusive_index = 0)
	{
		step_size_ = step_size;
		log_path_ = log_path;
		Initialize(N, s, shard, LCS_out);
		exclusive_index_ = exclusive_index;
	};
	// Only use this constructor after prebuilding case. This constructor need "Initialize" function after calling. 
	RankingManager(int step_size, 
		string& log_path,
		int exclusive_index = 0) 
	{
		step_size_ = step_size;
		log_path_ = log_path;
		exclusive_index_ = exclusive_index;
	}
	~RankingManager() {};

	// Initialize function for using pre-reconstruction result
	void Initialize(int N,
		int s,
		vector<Geom>& shard,
		list<LCSIndex>& LCS_out);

	void getShard(vector<Geom>& shard);

	void BuildStep(void);

	bool BuildGraph(RankingSubgraph& toprank_graph, int ext_index);

	bool EndCondition(RankingSubgraph& toprank_graph);
	
	vector<Step> step_;							// Structure of each step
	vector<RankingSubgraph> out_graph_;			// Output graph
	list<LCSIndex> initial_graph_;				// Initial graph, 
	vector<Geom> shard_; 
	vector<RankingSubgraph> previous_graph_;
	string log_path_;

private:
	int N_;		// The number of the preserved subgraphs
	int s_;		// THe number of searching at node priority list
	int step_counter_;
	int step_size_;
	MatrixXd GT_;

	// 0 : There is no removal, 1 : Reconstruct only rim parts, 2 : Reconstruct only base parts
	int exclusive_index_;	
};

struct PTR_PriorityList {
	int root_num_;
	int graph_index_;
	int priority_index_;
	int inlier_;
};

class State {
public:
	State() : state_score_(0) {};
	State(int num_shard) : state_score_(0), true_node_(num_shard, false) { };
	~State() {};

	// Including all graphs make priority list 
	void MakeTotalPriority(void);		

	// Fill out reconstructed shards into true_node_
	void SynchronizeTrueNode(void);		

	// Fill out reconstructed points as true value into total_matched_matrix
	void UpdateMatchedMatrix(vector<Geom>& shard);		

	// Update Statescore, which is sum of all graph score
	void UpdateStateScore(void);			

	bool isSimilarState(State& input);

	void SaveLog(const string& path, 
		int index,
		const vector<Trans>& T_axis);

public:
	vector<RankingSubgraph> graph_;
	vector<PTR_PriorityList> total_priority_;
	vector<bool> true_node_;
	vector<int> node_log_;
	int state_score_;
	vector<RankingSubgraph> pre_graph_;

	// e.g. if shard[0].point_.col(3) is matched then, matched_index_[0][3] is 'true' 
	vector<vector<bool>> total_matched_index_;	
};

class StateManager {
public:
	StateManager() : N_(0), s_(0), step_counter_(0) {};
	StateManager(int step_size, string& log_path) : N_(0), s_(0) {
		step_size_ = step_size;
		log_path_ = log_path;
	};

	void getShard(vector<Geom>& shard);

	void InitializeWithGraph(int N,
		int s,
		vector<Geom>& shard, 
		vector<RankingSubgraph>& pre_graph);

	void BuildStep(void);

	bool BuildState(State& state, 
		int order_index, 
		int ext_index);

	bool EndCondition(State& state);

	void PrepareNextStep(State& state);

	vector<State> ext_state_;			// extended state
	vector<State> out_state_;			// Output state
	list<LCSIndex> initial_graph_;		// Initial graph, 
	vector<Geom> shard_;
	string log_path_;

private:
	int N_;
	int s_;
	int step_counter_;
	int step_size_;
};

//############################## Other Functions ##############################//
bool InlierCompare(const Chunk& A,
	const Chunk& B);

void CalculateMatchingScore(Chunk& chunk, 
	const vector<LCSIndex>& lcs_out);

bool SortPair(const pair<int, int>& a, 
	const pair<int, int>& b);

vector<pair<int, int>> SortRoot(list<LCSIndex>& lcs_out, vector<Geom>& shard);

bool NodeCompare(const Chunk& A, 
	const Chunk& B);

// Pick only index edge, which is usually rim(index 1) or base(index 2) or fractured base(index 3)
void ExclusivelyPickEdge(list<LCSIndex>& lcs_out, 
	vector<Geom>& shard,
	int exclusive_index = 0);	

void TransAverage(int current_node, 
	vector<LCSIndex>& edges, 
	Matrix3d& R,
	Vector3d& t);

void PickEdges(list<LCSIndex>& lcs_out,
	const vector<bool>& true_node);

void SaveGraphLog(RankingSubgraph& graph, 
	const string& path, 
	int index, 
	const vector<Trans>& T_axis,
	const struct tm* curr_tm);

void RemoveEdgeUsingPCInlier(vector<Geom>& shard,
	RankingSubgraph& graph);

void RemoveOverlappedLCS(const RankingSubgraph& top_graph,
	list<LCSIndex>& lcs_out);

void RemoveSmallShards(vector<Geom>& shard, 
	list<LCSIndex>& lcs,
	int threshold = 50);

bool CompExtGraph(const RankingSubgraph& a,
	const RankingSubgraph& b);

void EditLCSPairTrans(RankingSubgraph& subgraph);

void RemoveSameGraph(vector<RankingSubgraph>& g_in,
	vector<RankingSubgraph>& g_out);
void RemoveSameGraph(vector<RankingSubgraph>& g);

void RemoveSameState(vector<State>& state);

bool isSameGraph(RankingSubgraph& g_a, 
	RankingSubgraph& g_b);

void PrepareGraphBuilding(vector<Geom>& shard,
	RankingSubgraph& toprank_graph);

void GraphAxisRefinement(vector<Geom>& shard,
	RankingSubgraph& toprank_graph);

bool CheckGraphPlausibility(vector<Geom>& shard,
	RankingSubgraph& toprank_graph,
	string& log_path, 
	int step_counter, 
	int ext_index);

#endif // !_RANKING_SYSTEM_H