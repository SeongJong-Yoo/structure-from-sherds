#include "ranking_system.h"
//############################## class RankingSubgraph ##############################//
void RankingSubgraph::InputTransLog(int index,
	Matrix3d& R,
	Vector3d& t)
{
	Matrix4d T_input = Matrix4d::Identity(), T;
	for (int i = 0; i < 3; i++) {
		T_input.row(i) << R(i, 0), R(i, 1), R(i, 2), t[i];
	}

	int start = 4 * (index - 1);
	T = T_log_.back().block(0, start, 4, 4);
	T = T_input * T;

	for (int i = start; i < start + 4; i++) {
		T_log_.back().col(i) = T.col(i - start);
	}
}

void RankingSubgraph::MakeTransLog(void)
{
	int shard_num = node_.size();
	MatrixXd T(4, 4 * shard_num);
	Matrix4d Id = Matrix4d::Identity();
	for (int i = 0; i < shard_num; i++) {
		int start = 4 * i;
		for (int j = start; j < start + 4; j++) {
			T.col(j) = Id.col(j - start);
		}
	}
	T_log_.push_back(T);
}

vector<string> RankingSubgraph::StringOutTransLog(int index, const vector<Trans>& T_axis)
{
	vector<string> out;

	for (int i = 0; i < 4; i++) {
		string contents;
		for (int j = 0; j < knum_of_shard_; j++) {
			int start = 4 * j;

			if (T_log_[index].block(0, start, 4, 4) == Matrix4d::Identity())
				continue;

			Matrix4d current_T_axis;
			T_axis[j].Output(current_T_axis);
			Matrix4d out_T = current_T_axis;	// Transformation of Z-axis alignment

			if (index < T_log_.size() - 1) {
				for (int k = 0; k <= index; k++) {
					out_T = T_log_[k].block(0, start, 4, 4) * out_T;
				}
			}
			else if (index == T_log_.size() - 1) {
				out_T = T_log_[index].block(0, start, 4, 4) * out_T;
			}

			if (i == 0) {
				contents += to_string(j + 1) + ", ";
			}
			else {
				contents += ", ";
			}
			for (int k = 0; k < 4; k++) {
				contents += to_string(out_T(i, k)) + ", ";
			}
		}
		contents += "\n";
		out.push_back(contents);
	}
	
	return out;
}

void RankingSubgraph::Copy(RankingSubgraph& input)
{
	edge_.clear();
	edge_.assign(input.edge_.begin(), input.edge_.end());
	edge_log_.clear();
	edge_log_.assign(input.edge_log_.begin(), input.edge_log_.end());
	edge_size_.clear();
	edge_size_.assign(input.edge_size_.begin(), input.edge_size_.end());
	T_log_.clear();
	T_log_.assign(input.T_log_.begin(), input.T_log_.end());
	node_log_.clear();
	node_log_.assign(input.node_log_.begin(), input.node_log_.end());
	node_.clear();
	node_.assign(input.node_.begin(), input.node_.end());
	graph_score_ = input.graph_score_;
	root_node_ = input.root_node_;
	T_.clear();
	T_.assign(input.T_.begin(), input.T_.end());

}


bool RankingSubgraph::isSimilarGraph(RankingSubgraph& G_i)
{
	double rad_threshold = 0.262, t_threshold = 20.0;	

	// First, check true node 
	int num_shard = this->node_.size();
	for (int i = 0; i < num_shard; i++) {
		if (this->node_[i] != G_i.node_[i]) {
			return false;
		}
	}
	int base_node = this->root_node_ - 1;

	Matrix4d T_base_A, T_base_B;
	this->T_[base_node].InvOut(T_base_A);
	G_i.T_[base_node].InvOut(T_base_B);

	for (int i = 0; i < num_shard; i++) {
		if (this->node_[i] && i != base_node) {
			Matrix4d T_a, T_b, T_a_i, T_b_i;
			this->T_[i].Output(T_a);
			G_i.T_[i].Output(T_b);

			T_a_i = T_base_A * T_a;
			T_b_i = T_base_B * T_b;
			if (!isSimilarTrans(T_a_i, T_b_i, rad_threshold, t_threshold)) {
				return false;
			}
		}
	}
	return true;
}

int RankingSubgraph::NumTrueNode(void)
{
	int out(0);
	for (int i = 0; i < node_.size(); i++) {
		if (node_[i])
			out++;
	}
	return out;
}


void RankingSubgraph::MakePriorityList(const vector<bool>& true_node)
{
	priority_list_.clear();
	vector<Chunk>().swap(priority_list_);
	//########## Clear subgraph lcs list
	sub_graph_.clear();
	vector<LCSIndex>().swap(sub_graph_);

	sub_graph_ = PickEdges(true_node);
	list<Chunk> priority;
	int num_lcs_list = sub_graph_.size();

	int count(0);
	for (int i = 0; i < num_lcs_list; i++) {
		Chunk dummy;
		int s_x = sub_graph_[i].shard_x_;
		int s_y = sub_graph_[i].shard_y_;
		if (true_node[s_x - 1]) dummy.node = s_y;
		else if (true_node[s_y - 1]) dummy.node = s_x;
		else cout << "Function : PickEdge is not working well" << endl;

		dummy.i_edge.emplace_back(count);
		priority.emplace_back(dummy);
		count++;
	}

	priority.sort(NodeCompare);

	CombineChunk(priority, sub_graph_);

	for (auto iter = priority.begin(); iter != priority.end(); ++iter) {
		CalculateMatchingScore(*iter, sub_graph_);
		priority_list_.emplace_back(*iter);
	}

	sort(priority_list_.begin(), priority_list_.end(), InlierCompare);
	priority_index_ = 0;
}

void RankingSubgraph::CombineChunk(list<Chunk>& priority, vector<LCSIndex>& lcs)
{
	for (auto iter = priority.begin(); iter != priority.end(); ++iter) {
		list<Chunk>::iterator c_iter = iter;
		while (c_iter != priority.end()) {
			if (iter->node != c_iter->node)
			{
				break;
			}
			else if (iter == c_iter) {
				++c_iter;
				continue;
			}
			else {
				int i_index = iter->i_edge[0];
				int c_index = c_iter->i_edge[0];
				int i_trans_from = lcs[i_index].trans_.index_;
				int i_trans_to = lcs[i_index].trans_.toward_;
				int c_trans_from = lcs[c_index].trans_.index_;
				int c_trans_to = lcs[c_index].trans_.toward_;
				//########## If translation of iter and c_iter is similar, then merge it.
				Matrix4d T_c, T_i;
				if ((iter->node == i_trans_from) && (iter->node == c_trans_from)) {
					lcs[i_index].trans_.Output(T_i);
					lcs[c_index].trans_.Output(T_c);
				}
				else if ((iter->node == i_trans_from) && (iter->node == c_trans_to)) {
					lcs[i_index].trans_.Output(T_i);
					lcs[c_index].trans_.InvOut(T_c);
				}
				else if ((iter->node == i_trans_to) && (iter->node == c_trans_from)) {
					lcs[i_index].trans_.InvOut(T_i);
					lcs[c_index].trans_.Output(T_c);
				}
				else if ((iter->node == i_trans_to) && (iter->node == c_trans_to)) {
					lcs[i_index].trans_.InvOut(T_i);
					lcs[c_index].trans_.InvOut(T_c);
				}
				else {
					cout << "Error : CombineChunck - Wrong compare case" << endl;
					T_c = Matrix4d::Zero();
					T_i = Matrix4d::Ones();
				}
				if (isSimilarTrans(T_c, T_i, 0.436, 20.0))
				{
					for (int i = 0; i < c_iter->i_edge.size(); i++) {
						//########## If two lcs are not same edges
						if (!(lcs[i_index].SamePart(lcs[c_index]))) {
							iter->i_edge.emplace_back(c_iter->i_edge[i]);
						}
					}
					c_iter = priority.erase(c_iter);
				}
				else {
					++c_iter;
				}
			}
		}
	}
}

void RankingSubgraph::ReplaceLCS(list<LCSIndex>& lcs_in)
{
	lcs_reference_.clear();
	list<LCSIndex>::iterator iter = lcs_in.begin();
	for (; iter != lcs_in.end(); iter++) {
		lcs_reference_.emplace_back(*iter);
	}
}

int RankingSubgraph::NodeOut(void)
{
	return priority_list_[priority_index_].node;
}

vector<LCSIndex> RankingSubgraph::EdgeOut(void)
{
	vector<LCSIndex> edge_out;

	int num = priority_list_[priority_index_].i_edge.size();

	for (int i = 0; i < num; i++) {
		int edge_index = priority_list_[priority_index_].i_edge[i];
		edge_out.emplace_back(sub_graph_[edge_index]);
	}

	return edge_out;
}

vector<LCSIndex> RankingSubgraph::PickEdges(const vector<bool>& true_node)
{
	vector<LCSIndex> lcs_input;
	list<LCSIndex>::iterator iter = lcs_reference_.begin();
	for (; iter != lcs_reference_.end(); ++iter) {
		int s_x = iter->shard_x_ - 1;
		int s_y = iter->shard_y_ - 1;
		if ((!true_node[s_x]) != (!true_node[s_y])) {
			lcs_input.emplace_back(*iter);
		}
	}
	return lcs_input;
}

void RankingSubgraph::PickEdges(const vector<bool>& true_node,
	vector<LCSIndex>& inside_lcs,
	vector<LCSIndex>& outside_lcs)
{
	list<LCSIndex>::iterator iter = lcs_reference_.begin();
	for (; iter != lcs_reference_.end(); ++iter) {
		int s_x = iter->shard_x_ - 1;
		int s_y = iter->shard_y_ - 1;
		if ((!true_node[s_x]) != (!true_node[s_y])) {
			inside_lcs.emplace_back(*iter);
		}
		else {
			outside_lcs.emplace_back(*iter);
		}
	}
}


//############################## class RankingManager ##############################//
void RankingManager::Initialize(int N,
	int s, 
	vector<Geom>& shard,
	list<LCSIndex>& LCS_out)
{
	cout << "----- Initialize Building graph  -----" << endl;
	N_ = N;
	s_ = s;
	step_counter_ = 0;
	step_.clear();
	out_graph_.clear();
	initial_graph_.clear();
	initial_graph_ = LCS_out;

	getShard(shard);

	vector<pair<int, int>> root_node = SortRoot(initial_graph_, shard_);

	int num_shard = shard.size();

	Step step_0;

	cout << "----- Make root graph  -----" << endl;
	int num_s = root_node.size();
	if (num_s > s_) num_s = s_;
	for (int i = 0; i < num_s; i++) {
		if (root_node[i].second <= 0) {
			continue;
		}
		RankingSubgraph top_rank_graph(initial_graph_, num_shard);

		top_rank_graph.node_[root_node[i].first - 1] = true; // first : node index
		top_rank_graph.root_node_ = root_node[i].first;
		top_rank_graph.node_log_.push_back(root_node[i].first);
		top_rank_graph.MakePriorityList(top_rank_graph.node_);

		step_0.top_rank_graph_.push_back(top_rank_graph);
	}


	step_.push_back(step_0);
}

void RankingManager::getShard(vector<Geom>& shard)
{
	shard_.clear();
	int num_shard = shard.size();
	for (int i = 0; i < num_shard; i++) {
		shard_.push_back(shard[i]);
	}
}

void RankingManager::BuildStep(void)
{
	cout << "----- Start : " << step_counter_ << " Step -----" << endl;

	int searching_weight = 1;

	int num_subgraph = step_.back().top_rank_graph_.size();
	int num_shard = shard_.size();
	Step next_step;

	cout << "----- Expend subgraphs Select " << searching_weight * s_ << " number of graphs" << endl;
	cout << "Number of current subgraph : " << num_subgraph << " same or less than " << N_ << endl;

	for (int i = 0; i < num_subgraph; i++) {
		if (EndCondition(step_.back().top_rank_graph_[i])) {
			out_graph_.push_back(step_.back().top_rank_graph_[i]);
			continue;
		}
		else {
			int ext_counter(0);
			for (int j = 0; j < searching_weight * s_; j++) {
				RankingSubgraph next_sub = step_.back().top_rank_graph_[i];
				if (j > next_sub.priority_list_.size() - 1) {
					break;
				}
				next_sub.priority_index_ = j;
				if (BuildGraph(next_sub, i)) {
					next_step.ext_graph_.push_back(next_sub);
					ext_counter++;
				}
			}
		}
	}
	step_counter_++;

	if (next_step.ext_graph_.empty()) {
		cout << "----- Finish build graph  -----" << endl;
		//########## Finish this alogrithm and sort the output graph
		RemoveSameGraph(out_graph_);

		sort(out_graph_.begin(), out_graph_.end(), CompExtGraph);
		for (int i = 0; i < out_graph_.size(); i++) {
			out_graph_[i].MakeTransLog();
			Matrix3d R;
			Vector3d t;
			for (int j = 0; j < out_graph_[i].T_.size(); j++) {
				out_graph_[i].T_[j].Output(R, t);
				out_graph_[i].InputTransLog(j + 1, R, t);
			}
		}
		return;
	}

	else {
		cout << "----- Select N subgraphs for next step  -----" << endl;
		std::sort(next_step.ext_graph_.begin(), next_step.ext_graph_.end(), greater<RankingSubgraph>());

		vector<RankingSubgraph> ext_pruned;
		RemoveSameGraph(next_step.ext_graph_, ext_pruned);

		int N = ext_pruned.size();
		if (N > N_) N = N_;

		cout << "Start to pick " << N << " number of high rank graph" << endl;
		for (int iter = 0; iter < N; iter++) {	
			int root_node = ext_pruned[iter].root_node_ - 1;

			ext_pruned[iter].priority_list_.clear();
			vector<Chunk>().swap(ext_pruned[iter].priority_list_);

			//########## Restore shards position
			for (int j = 0; j < num_shard; j++) {
				if (ext_pruned[iter].node_[j]) {
					Matrix3d R;
					Vector3d t;
					ext_pruned[iter].T_[j].Output(R, t);
					shard_[j].Move(R, t);
				}
			}
			for (int k = 0; k < num_shard; k++) {
				int num_axis = shard_[k].edge_line_.axis_norm_.size();
				if ((num_axis == 1) || (ext_pruned[iter].node_[k])) {
					CalculateFeatureAxisless(shard_[k]);
				}
				else {
					CalculateFeatureAxisless(shard_[k], 0);
					for (int j = 1; j < num_axis; j++) {
						Matrix3d R_a, R_i;
						Vector3d t_a, t_i;
						AxisAlignment(shard_[k].edge_line_, R_a, t_a, j);
						CalculateFeatureAxisless(shard_[k], j);
						R_i = R_a.inverse();
						t_i = -R_i * t_a;
						shard_[k].MoveWOSurface(R_i, t_i);
					}
				}
			}

			//########## Update feature matching result Feature recalculate
			list<LCSIndex> lcs_out; 

			//########## Feature recalculate
			FeatureCompGraphBuilding(shard_, 
				lcs_out,
				25, 
				MINIMUM_NUMBER, 
				ext_pruned[iter].matched_index_); 

			ExclusivelyPickEdge(lcs_out, shard_, exclusive_index_);
			PickEdges(lcs_out, ext_pruned[iter].node_);
			RegistrationPruning(shard_, lcs_out, ext_pruned[iter].node_);

			ext_pruned[iter].ReplaceLCS(lcs_out);

			RemoveEdgeUsingPCInlier(shard_, ext_pruned[iter]);

			//########## Make priority list
			ext_pruned[iter].MakePriorityList(ext_pruned[iter].node_);
			next_step.top_rank_graph_.push_back(ext_pruned[iter]);

		}
		step_.push_back(next_step);
		vector<RankingSubgraph>().swap(ext_pruned);

		BuildStep();
		return;
	}
}

bool RankingManager::BuildGraph(RankingSubgraph& toprank_graph, int ext_index)
{
	int num_shard = shard_.size();
	bool return_value = true;

	PrepareGraphBuilding(shard_, toprank_graph);

	//########## Save initial axis data
	Vector3d axis_point_log(shard_[toprank_graph.root_node_ - 1].edge_line_.axis_point_[0]);
	Vector3d axis_norm_log(shard_[toprank_graph.root_node_ - 1].edge_line_.axis_norm_[0]);

	vector<Matrix3d> R(num_shard);
	vector<Vector3d> t(num_shard);
	for (int i = 0; i < num_shard; i++) {
		R[i] = Matrix3d::Identity();
		t[i] = { 0, 0, 0 };
	}
	int global_inlier = 0;
	bool rim_restrain = true, axis_restrain = true;

	IcpIncGraphAxis(shard_,
		R,
		t,
		toprank_graph,
		previous_graph_, 
		toprank_graph.graph_score_, 
		rim_restrain, axis_restrain);

	vector<Geom*> geom_ptr;
	for (int i = 0; i < num_shard; i++) {
		toprank_graph.T_[i].Input(R[i], t[i]);
		toprank_graph.InputTransLog(i + 1, R[i], t[i]);
	}
	GraphAxisRefinement(shard_, toprank_graph);

	return_value = CheckGraphPlausibility(shard_, 
						toprank_graph,
						log_path_, 
						step_counter_,
						ext_index);

	for (int i = 0; i < num_shard; i++) {
		Matrix3d R_i;
		Vector3d t_i;
		if (toprank_graph.node_[i]) {
			toprank_graph.T_[i].InvOut(R_i, t_i);
			shard_[i].Move(R_i, t_i);
		}
	}
	shard_[toprank_graph.root_node_ - 1].edge_line_.axis_point_[0] = axis_point_log;
	shard_[toprank_graph.root_node_ - 1].edge_line_.axis_norm_[0] = axis_norm_log;

	return return_value;
}

bool RankingManager::EndCondition(RankingSubgraph& toprank_graph)
{
	bool out = false;
	int counter(0);
	for (int i = 0; i < shard_.size(); i++) {
		if (toprank_graph.node_[i]) counter++;
	}
	if (counter == shard_.size()) {
		cout << "End condition : All shards are matched" << endl;
		out = true;
	}

	if ((exclusive_index_ == 0) && (step_counter_ == step_size_)) {
		cout << "End condition : Counter Finish" << endl;

		out = true;
	}

	if (toprank_graph.priority_list_.empty()) {
		cout << "End condition : Priority list empty" << endl;
		out = true;
	}

	return out;
}

//############################## class State ##############################//
void State::MakeTotalPriority(void)
{
	total_priority_.clear();
	vector<PTR_PriorityList>().swap(total_priority_);

	for(int i = 0; i < graph_.size(); i++){
		int root_num = graph_[i].root_node_;
		PickEdges(graph_[i].lcs_reference_, true_node_);
		graph_[i].MakePriorityList(graph_[i].node_);
		for (int j = 0; j < graph_[i].priority_list_.size(); j++) {
			PTR_PriorityList ptr_P_L;
			ptr_P_L.root_num_ = root_num;
			ptr_P_L.graph_index_ = i;
			ptr_P_L.priority_index_ = j;
			ptr_P_L.inlier_ = graph_[i].priority_list_[j].inlier;
			total_priority_.push_back(ptr_P_L);
		}
	}
	sort(total_priority_.begin(), total_priority_.end(),
		[](PTR_PriorityList a, PTR_PriorityList b) -> bool {
		return a.inlier_ > b.inlier_;
		});
}

void State::SynchronizeTrueNode(void)
{
	true_node_.clear();
	vector<bool>().swap(true_node_);

	true_node_.assign(graph_[0].node_.begin(), graph_[0].node_.end());
	for (int i = 0; i < true_node_.size(); i++) {
		for (int j = 1; j < graph_.size(); j++) {
			if (graph_[j].node_[i])
				true_node_[i] = true;
		}
	}
}

void State::UpdateMatchedMatrix(vector<Geom>& shard)
{
	if (total_matched_index_.empty()) {
		total_matched_index_.resize(shard.size());
		for (int i = 0; i < shard.size(); i++) {
			vector<bool> m_index(shard[i].edge_line_.point_.cols(), false);
			total_matched_index_[i] = m_index;
		}
	}
	for (int i = 0; i < total_matched_index_.size(); i++) {
		for (int j = 0; j < total_matched_index_[i].size(); j++) {
			total_matched_index_[i][j] = false;
			for (int k = 0; k < graph_.size(); k++) {
				if ((!graph_[k].matched_index_.empty()) && (graph_[k].matched_index_[i][j])) {
					total_matched_index_[i][j] = true;
					break;
				}
			}
		}
	}
}

void State::UpdateStateScore(void)
{
	this->state_score_ = 0;
	for (int i = 0; i < this->graph_.size(); i++) {
		this->state_score_ += this->graph_[i].graph_score_;
	}
}

bool State::isSimilarState(State& input)
{
	bool out = true;
	
	for (int i = 0; i < graph_.size(); i++) {
		for (int j = 0; j < input.graph_.size(); j++) {
			if (graph_[i].root_node_ == input.graph_[j].root_node_) {
				if (!graph_[i].isSimilarGraph(input.graph_[j])) {
					out = false;
					break;
				}
			}			
		}
		if (!out) {
			break;
		}
	}

	return out;
}

void State::SaveLog(const string& path,
	int index,
	const vector<Trans>& T_axis)
{
	//########## Get time information for saving ##########//
	struct tm* curr_tm;
	time_t curr_time = time(nullptr);
	curr_tm = localtime(&curr_time);
	int year = curr_tm->tm_year + 1900;
	int mon = curr_tm->tm_mon + 1;
	int day = curr_tm->tm_mday;
	int hour = curr_tm->tm_hour;
	int min = curr_tm->tm_min;

	int graph_size = this->graph_.size();
	string log_path = path + "_Top_" + to_string(index + 1) + "_history_" 
		+ to_string(year) + "_" + to_string(mon) + "_" + to_string(day)
		+ "_" + to_string(hour) + "_" + to_string(min) + ".txt";
	ofstream writeStream(log_path, ios::out | ios::trunc);
	if (writeStream) {
		writeStream << "Root Node : ";
		for (int i = 0; i < graph_.size() - 1; i++) {
			writeStream << graph_[i].root_node_ << ", ";
		}
		writeStream << graph_.back().root_node_ << endl;
		writeStream << "Base Restored Node Order : ";
		for (int i = 0; i < pre_graph_.size(); i++) {
			writeStream << pre_graph_[i].root_node_ << " -> ";
			for (int j = 1; j < pre_graph_[i].node_log_.size() - 1; j++) {
				writeStream << pre_graph_[i].node_log_[j] << " -> ";
			}
			writeStream << pre_graph_[i].node_log_.back() << endl;
		}
		if (pre_graph_.empty()) {
			writeStream << "None" << endl;
		}

		writeStream << "After Base, Restored Node Order : ";
		for (int i = 0; i < node_log_.size(); i++) {
			writeStream << " -> " << node_log_[i];
		}
		writeStream << endl;
		writeStream << "Score : " << state_score_ << endl;
	}
	if (writeStream) {
		writeStream.close();
	}
	for (int i = 0; i < graph_size; i++) {
		string out_path = path + "_Top_" + to_string(index + 1) + "_Graph_" + to_string(i + 1) + "_";
		SaveGraphLog(this->graph_[i], out_path, i, T_axis, curr_tm);
	}
}

//############################## class StateManager ##############################//
void StateManager::getShard(vector<Geom>& shard)
{
	shard_.clear();
	int num_shard = shard.size();
	for (int i = 0; i < num_shard; i++) {
		shard_.push_back(shard[i]);
	}
}

void StateManager::InitializeWithGraph(int N,
	int s,
	vector<Geom>& shard, 
	vector<RankingSubgraph>& pre_graph)
{
	cout << "----- Initialize Building graph including pre-graph  -----" << endl;
	N_ = N;
	s_ = s;
	step_counter_ = 0;
	ext_state_.clear();
	out_state_.clear();
	initial_graph_.clear();
	getShard(shard);
	int num_shard = shard.size();

	State root_state;
	int num_pre_graph = pre_graph.size();
	for (int i = 0; i < num_pre_graph; i++) {
		root_state.graph_.push_back(pre_graph[i]);
		root_state.pre_graph_.push_back(pre_graph[i]);
	}
	for (int i = 0; i < num_shard; i++) {
		if (shard_[i].edge_line_.is_seg_base_ && shard_[i].edge_line_.is_sane_base_) {
			RankingSubgraph root_graph(initial_graph_, num_shard);

			root_graph.node_[i] = true;
			root_graph.root_node_ = i + 1;
			root_graph.node_log_.push_back(i + 1);
			root_state.graph_.push_back(root_graph);
		}
	}
	root_state.SynchronizeTrueNode();
	root_state.UpdateMatchedMatrix(shard_);
	root_state.UpdateStateScore();

	vector<bool> true_node(num_shard, false);
	//################## Pre graph error check
	for (int i = 0; i < num_pre_graph; i++) {
		for (int j = 0; j < num_shard; j++) {
			if (pre_graph[i].node_[j]) {
				if (true_node[j]) {
					cout << "StateManager::InitializeWithGraph Error : There is repeated shard" << endl;
					break;
				}
				else {
					true_node[j] = true;
					if(!root_state.true_node_[j])
						cout << "StateManager::InitializeWithGraph Error : True node is not synchronized with pre_graph's one" << endl;
				}
			}
		}
	}
	PrepareNextStep(root_state);
	ext_state_.push_back(root_state);
}

bool StateManager::EndCondition(State& state)
{
	bool out = false;

	int counter(0);
	for (int i = 0; i < shard_.size(); i++) {
		if (state.true_node_[i])
			counter++;
	}
	if (counter == shard_.size()) {
		cout << "End condition : All shards are matched" << endl;
		out = true;
	}
	if (!out) {
		if (step_counter_ == step_size_) {
			cout << "End condition : Counter Finish" << endl;
			out = true;
		}
	}

	int p_l_counter(0);
	for (int i = 0; i < state.graph_.size(); i++) {
		if (state.graph_[i].priority_list_.empty())
			p_l_counter++;
	}
	if (p_l_counter == state.graph_.size()) {
		cout << "End condition : Priority list empty" << endl;
		out = true;
	}

	return out;
}

void StateManager::PrepareNextStep(State& state)
{
	int num_shard = shard_.size();
	for (int i = 0; i < state.graph_.size(); i++) {
		//########## Clear previous priority list
		state.graph_[i].priority_list_.clear();
		vector<Chunk>().swap(state.graph_[i].priority_list_);

		//########## Restore shards position.
		for (int j = 0; j < num_shard; j++) {
			if (state.graph_[i].node_[j]) {
				Matrix3d R; Vector3d t;
				state.graph_[i].T_[j].Output(R, t);
				shard_[j].Move(R, t);
			}
		}
	}

	//########## Update axis based feature
	for (int j = 0; j < num_shard; j++) {
		int num_axis = shard_[j].edge_line_.axis_norm_.size();
		if ((num_axis == 1) || (state.true_node_[j])) {
			CalculateFeatureAxisless(shard_[j], 0);
		}
		else {
			CalculateFeatureAxisless(shard_[j], 0);
			for (int k = 1; k < num_axis; k++) {
				Matrix3d R_a, R_i;
				Vector3d t_a, t_i;
				AxisAlignment(shard_[j].edge_line_, R_a, t_a, k);
				CalculateFeatureAxisless(shard_[j], k);
				R_i = R_a.inverse();
				t_i = -R_i * t_a;
				shard_[j].MoveWOSurface(R_i, t_i);
			}
		}
	}

	//########## Make new 'edge'
	list<LCSIndex> lcs_out;
	FeatureCompGraphBuilding(shard_, 
					lcs_out,
					25, 
					MINIMUM_NUMBER,
					state.total_matched_index_);

	PickEdges(lcs_out, state.true_node_);

	for (int i = 0; i < state.graph_.size(); i++) {
		list<LCSIndex> lcs = lcs_out;
		PickEdges(lcs, state.graph_[i].node_);
		RegistrationPruning(shard_, lcs, state.graph_[i].node_);
		state.graph_[i].ReplaceLCS(lcs);

		RemoveEdgeUsingPCInlier(shard_, state.graph_[i]);
	}
	state.MakeTotalPriority();
}


void StateManager::BuildStep(void)
{
	cout << "----- Start : " << step_counter_ << " Step -----" << endl;

	int num_state = ext_state_.size();
	int num_shard = shard_.size();
	vector<State> next_state_set;
	int ext_s = s_;
	cout << "----- Expend  Select " << ext_s << " number of graphs" << endl;
	cout << "Number of current States : " << num_state << " same or less than " << N_ << endl;

	//########## Extend State (FIRST STAGE) 
	//########## Current state manager has 'num_state' states, and each state explores 's_' times different states.
	for (int i = 0; i < num_state; i++) {
		//########## If state satisfies end condition, save state into 'out_state_' vector
		if (EndCondition(ext_state_[i])) {
			out_state_.push_back(ext_state_[i]);
			continue;
		}
		//########## Extend 's_' number of state based on priority list
		else {
			int ext_counter(0);
			for (int j = 0; j < ext_s; j++) {
				State next_state = ext_state_[i];
				if (j > next_state.total_priority_.size() - 1)
					break;
				else if (BuildState(next_state, j, i)) {
					next_state_set.push_back(next_state);
					ext_counter++;
				}
			}
			//########## If the extension failed, Show current state as result
			if (ext_counter == 0) {
				out_state_.push_back(ext_state_[i]);
			}
		}
	}
	step_counter_++;
	ext_state_.clear();
	vector<State>().swap(ext_state_);

	//########## End all graph building process
	if (next_state_set.empty()) {
		cout << "----- Finish build graph  -----" << endl;
		sort(out_state_.begin(), out_state_.end(), [](State a, State b) -> bool {
			return a.state_score_ > b.state_score_;
			});
		RemoveSameState(out_state_);

		for (int i = 0; i < out_state_.size(); i++) {
			for (int j = 0; j < out_state_[i].graph_.size(); j++) {
				out_state_[i].graph_[j].MakeTransLog();
				Matrix3d R; Vector3d t;
				for (int k = 0; k < out_state_[i].graph_[j].T_.size(); k++) {
					out_state_[i].graph_[j].T_[k].Output(R, t);
					out_state_[i].graph_[j].InputTransLog(k + 1, R, t);
				}
			}
		}
		return;
	}

	//########## Prepare next Step (SECOND STAGE)
	else {
		cout << "----- Select N subgraphs for next step  -----" << endl;
		sort(next_state_set.begin(), next_state_set.end(), [](State a, State b) -> bool {
			return a.state_score_ > b.state_score_;
			});
		RemoveSameState(next_state_set);

		int N = next_state_set.size();
		if (N > N_) N = N_;

		cout << "Start to pick" << N << " number of high rank state" << endl;
		for (int iter = 0; iter < N; iter++) {
			PrepareNextStep(next_state_set[iter]);
			ext_state_.push_back(next_state_set[iter]);
		}
	}
	next_state_set.clear();
	vector<State>().swap(next_state_set);


	BuildStep();
	return;
}

bool StateManager::BuildState(State& state,
	int order_index,
	int ext_index)
{
	bool return_value = true;
	int num_shard = shard_.size();

	int root_num = state.total_priority_[order_index].root_num_;
	int graph_index = state.total_priority_[order_index].graph_index_;
	if (state.graph_[graph_index].root_node_ != state.total_priority_[order_index].root_num_) {
		cout << "StateManager :: BuildState Graph doesn't accord record" << endl;
		return false;
	}
	state.graph_[graph_index].priority_index_ = state.total_priority_[order_index].priority_index_;
	int current_node = state.graph_[graph_index].NodeOut() - 1;
	state.true_node_[current_node] = true;
	state.node_log_.push_back(current_node + 1);

	//########## Save initial axis data
	Vector3d axis_point_log(shard_[state.graph_[graph_index].root_node_ - 1].edge_line_.axis_point_[0]);
	Vector3d axis_norm_log(shard_[state.graph_[graph_index].root_node_ - 1].edge_line_.axis_norm_[0]);

	int root_node = state.graph_[graph_index].root_node_ - 1;

	PrepareGraphBuilding(shard_, state.graph_[graph_index]);

	vector<Matrix3d> R(num_shard);
	vector<Vector3d> t(num_shard);
	for (int i = 0; i < num_shard; i++) {
		R[i] = Matrix3d::Identity();
		t[i] = { 0, 0, 0 };
	}
	int global_inlier = 0;
	bool rim_restrain = true, axis_restrain = true;
	vector<RankingSubgraph> Dummy_graph;

	IcpIncGraphAxis(shard_,
				R, 
				t,
				state.graph_[graph_index],
				Dummy_graph, 
				state.graph_[graph_index].graph_score_, 
				rim_restrain, 
				axis_restrain);

	vector<Geom*> geom_ptr;
	for (int i = 0; i < num_shard; i++) {
		state.graph_[graph_index].T_[i].Input(R[i], t[i]);
		state.graph_[graph_index].InputTransLog(i + 1, R[i], t[i]);
	}
	GraphAxisRefinement(shard_, state.graph_[graph_index]);


	return_value = CheckGraphPlausibility(shard_, 
						state.graph_[graph_index], 
						log_path_, 
						step_counter_, 
						ext_index);



	for (int i = 0; i < num_shard; i++) {
		Matrix3d R_i;
		Vector3d t_i;
		if (state.graph_[graph_index].node_[i]) {
			state.graph_[graph_index].T_[i].InvOut(R_i, t_i);
			shard_[i].Move(R_i, t_i);
		}
	}
	shard_[state.graph_[graph_index].root_node_ - 1].edge_line_.axis_point_[0] = axis_point_log;
	shard_[state.graph_[graph_index].root_node_ - 1].edge_line_.axis_norm_[0] = axis_norm_log;


	if (return_value) {
		state.UpdateMatchedMatrix(shard_);
		state.UpdateStateScore();
	}

	return return_value;
}

//############################## Other function ##############################//
bool InlierCompare(const Chunk& A, const Chunk& B)
{
	if (A.inlier == B.inlier) {
		if (A.node == B.node) {
			return A.i_edge > B.i_edge;
		}
		else {
			return A.node > B.node;
		}
	}

	else {
		return A.inlier > B.inlier;
	}
}

void CalculateMatchingScore(Chunk& chunk, const vector<LCSIndex>& lcs_out)
{
	// Reset the score
	chunk.inlier = 0;
	for (int i = 0; i < chunk.i_edge.size(); i++) {
		int index = chunk.i_edge[i];
		chunk.inlier += lcs_out[index].inliner_;
	}
}

bool SortPair(const pair<int, int>& a,
	const pair<int, int>& b)
{
	return a.second > b.second;
}

vector<pair<int, int>> SortRoot(list<LCSIndex>& lcs_out, vector<Geom>& shard)
{
	int num_of_shard = shard.size();
	vector<pair<int, int>> node(num_of_shard);	// first : node index, second : score

	list<LCSIndex>::iterator iter = lcs_out.begin();

	for (; iter != lcs_out.end(); ++iter) {
		node[iter->shard_x_ - 1].second += iter->inliner_;
		node[iter->shard_y_ - 1].second += iter->inliner_;
	}

	for (int i = 0; i < node.size(); i++) {
		double input_w = (double)shard[i].edge_line_.point_.cols() / 100;
		double volume_weight = (exp(input_w) - exp(-input_w)) / (exp(input_w) + exp(-input_w));
		node[i].second = (int)(volume_weight * (double)node[i].second);
		node[i].first = i + 1;
		if (shard[i].edge_line_.is_seg_base_ || shard[i].edge_line_.is_seg_rim_) {
			node[i].second = (int)(2.0 * (double)node[i].second);
		}
	}

	sort(node.begin(), node.end(), SortPair);

	cout << "Node Sequence : ";
	for (int i = 0; i < node.size(); i++) {
		cout << node[i].first << "(" << node[i].second << "), ";
	}
	cout << endl;
	return node;
}

bool NodeCompare(const Chunk& A, const Chunk& B)
{
	return A.node < B.node;
}


void ExclusivelyPickEdge(list<LCSIndex>& lcs_out,
	vector<Geom>& shard,
	int exclusive_index)
{
	if (exclusive_index == 0)
		return;

	list<LCSIndex>::iterator iter = lcs_out.begin();

	if (exclusive_index == 1) {
		for (; iter != lcs_out.end(); ) {
			int s_x = iter->shard_x_ - 1;
			int s_y = iter->shard_y_ - 1;
			if ((shard[s_x].edge_line_.is_seg_rim_) && (shard[s_y].edge_line_.is_seg_rim_)) {
				++iter;
			}
			else
				iter = lcs_out.erase(iter);
		}
	}
	else if (exclusive_index == 2) {
		for (; iter != lcs_out.end(); ) {
			int s_x = iter->shard_x_ - 1;
			int s_y = iter->shard_y_ - 1;
			if ((shard[s_x].edge_line_.is_seg_base_) && (shard[s_y].edge_line_.is_seg_base_)) {
				++iter;
			}
			else
				iter = lcs_out.erase(iter);
		}
	}
	else if (exclusive_index == 3) {
		for (; iter != lcs_out.end(); ) {
			int s_x = iter->shard_x_ - 1;
			int s_y = iter->shard_y_ - 1;
			if ((shard[s_x].edge_line_.is_seg_base_) && (shard[s_y].edge_line_.is_seg_base_)) {
				if ((!shard[s_x].edge_line_.is_sane_base_) && (!shard[s_y].edge_line_.is_sane_base_)) {
					++iter;
				}
				else {
					iter = lcs_out.erase(iter);
				}
			}
			else
				iter = lcs_out.erase(iter);
		}
	}
}

void TransAverage(int current_node,
	vector<LCSIndex>& edges,
	Matrix3d& R,
	Vector3d& t)
{
	//########## Choose one reference rotation. And calculate average relative rotation matrix
	//########## to avoid singularity
	int num_edge = edges.size();
	Vector3d  w = { 0, 0, 0 }, t_avg = { 0, 0, 0 };

	R = Matrix3d::Zero();

	Matrix4d T_ref_inv;
	Matrix3d R_ref, R_avg;
	Vector3d t_ref = { 0, 0, 0 };
	if (edges[0].trans_.index_ == current_node) {
		edges[0].trans_.InvOut(T_ref_inv);
		edges[0].trans_.Output(R_ref, t_ref);
	}
	else {
		edges[0].trans_.Output(T_ref_inv);
		edges[0].trans_.InvOut(R_ref, t_ref);
	}

	if (num_edge == 1) {
		R = R_ref;
		t = t_ref;
		return;
	}

	vector<Matrix3d> R_r;
	vector<Vector3d> t_r;
	for (int i = 1; i < num_edge; i++) {
		Matrix4d tmp_T, T_r;
		if (edges[i].trans_.index_ == current_node) {
			edges[i].trans_.Output(tmp_T);
		}
		else {
			edges[i].trans_.InvOut(tmp_T);
		}
		T_r = tmp_T * T_ref_inv;

		Matrix3d tmp_R = Matrix3d::Identity();
		Vector3d tmp_w, tmp_t = { 0, 0, 0 };
		for (int k = 0; k < 3; k++) {
			tmp_R.row(k) << T_r(k, 0), T_r(k, 1), T_r(k, 2);
			tmp_t[k] = T_r(k, 3);
		}

		tmp_R = tmp_R.log();
		tmp_w << -tmp_R(1, 2), tmp_R(0, 2), -tmp_R(0, 1);
		w += tmp_w;
		t_avg += tmp_t;
	}

	w = w / num_edge;
	t_avg = t_avg / num_edge;
	R_avg.col(0) << 0, w(2), -w(1);
	R_avg.col(1) << -w(2), 0, w(0);
	R_avg.col(2) << w(1), -w(0), 0;

	R_avg = R_avg.exp();

	R = R_avg * R_ref;
	t = R_avg * t_ref + t_avg;
}

void PickEdges(list<LCSIndex>& lcs_out, const vector<bool>& true_node)
{
	list<LCSIndex>::iterator iter = lcs_out.begin();

	for (; iter != lcs_out.end(); ) {
		int s_x = iter->shard_x_ - 1;
		int s_y = iter->shard_y_ - 1;
		if ((!true_node[s_x]) != (!true_node[s_y])) {
			++iter;
		}
		else {
			iter = lcs_out.erase(iter);
		}
	}
}

void SaveGraphLog(RankingSubgraph& graph,
	const string& path,
	int index,
	const vector<Trans>& T_axis,
	const struct tm* curr_tm)
{
	int year = curr_tm->tm_year + 1900;
	int mon = curr_tm->tm_mon + 1;
	int day = curr_tm->tm_mday;
	int hour = curr_tm->tm_hour;
	int min = curr_tm->tm_min;

	string out_path = path + to_string(year) + "_" + to_string(mon) 
		+ "_" + to_string(day) + "_" + to_string(min);
	out_path += ".csv";
	ofstream writeStream(out_path, ios::out | ios::trunc);
	if (writeStream) {
		writeStream << "Node : ";
		for (int i = 0; i < graph.node_log_.size(); i++) {
			writeStream << "->" << graph.node_log_[i];
		}
		writeStream << endl;
		writeStream << "Score : " << graph.graph_score_ << endl;
		writeStream << "Step, ";
		writeStream << "Node, ";
		writeStream << "Edge, ";
		writeStream << "Transformation" << endl;
	}

	int counter(0);
	int num_shard = graph.node_.size();
	for (int i = 1; i < graph.node_log_.size(); i++) {
		int num_edge = graph.edge_size_[i - 1];
		int for_j = max(num_edge, 4);
		int edge_counter(0);
		int s_x = graph.edge_log_[counter].first, s_y = graph.edge_log_[counter].second;

		vector<string> T_string = graph.StringOutTransLog(i - 1, T_axis);

		writeStream << i << ", ";					// Step
		writeStream << graph.node_log_[i] << ", ";	// Node
		writeStream << s_x << "-" << s_y << ", ";	// Edge
		writeStream << T_string[0];

		for (int j = 1; j < for_j; j++) {
			if (j < num_edge) {
				counter++;
				s_x = graph.edge_log_[counter].first, s_y = graph.edge_log_[counter].second;
				writeStream << ", " << ", " << s_x << "-" << s_y << ", ";
			}
			else {
				writeStream << ", " << ", " << ", ";
			}
			if (j < 4) {
				writeStream << T_string[j];
			}
			else writeStream << endl;
		}
		writeStream << endl;
		counter++;
	}

	vector<string> T_string = graph.StringOutTransLog(graph.T_log_.size() - 1, T_axis);
	for (int i = 0; i < 4; i++) {
		writeStream << T_string[i];
	}

	if (writeStream) {
		writeStream.close();
	}
	else {
		cout << "Ranking_System : File opening error" << endl;
	}
}

void RemoveOverlappedLCS(const RankingSubgraph& top_graph, list<LCSIndex>& lcs_out)
{
	list<LCSIndex>::iterator iter = lcs_out.begin();
	for (; iter != lcs_out.end(); ) {

		int size_A = iter->end_.y - iter->start_.y;	// Correct order case
		int s_A = iter->shard_y_ - 1;

		int counter_A(0);
		for (int i = iter->start_.y; i <= iter->end_.y; i++) {
			if (top_graph.matched_index_[s_A][i]) {
				counter_A++;
			}
		}

		int size_B = iter->end_.x - iter->start_.x;	// Inverse order case
		int s_B = iter->shard_x_ - 1;
		int counter_B(0);
		int start_B = top_graph.matched_index_[s_B].size() - iter->end_.x - 1;
		int end_B = top_graph.matched_index_[s_B].size() - iter->start_.x - 1;
		for (int i = start_B; i <= end_B; i++) {
			if (top_graph.matched_index_[s_B][i]) {
				counter_B++;
			}
		}

		if ((counter_A > MINIMUM_NUMBER) || (counter_B > MINIMUM_NUMBER)) {
			iter = lcs_out.erase(iter);
		}
		else {
			iter++;
		}
	}
}

void RemoveSmallShards(vector<Geom>& shard, list<LCSIndex>& lcs, int threshold)
{
	list<LCSIndex>::iterator iter = lcs.begin();

	for (; iter != lcs.end(); ) {
		int num_x = shard[iter->shard_x_ - 1].edge_line_.point_.cols();
		int num_y = shard[iter->shard_y_ - 1].edge_line_.point_.cols();
		if ((num_x < threshold) || (num_y < threshold)) {
			iter = lcs.erase(iter);
		}
		else
			iter++;
	}
}
bool CompExtGraph(const RankingSubgraph& a, const RankingSubgraph& b)
{
	return a.graph_score_ > b.graph_score_;
}

void EditLCSPairTrans(RankingSubgraph& subgraph)
{
	list<LCSIndex>::iterator iter = subgraph.lcs_reference_.begin();
	for (; iter != subgraph.lcs_reference_.end(); ) {
		int index = iter->trans_.index_;
		int toward = iter->trans_.toward_;
		if (subgraph.node_[index - 1]) {
			Matrix4d T_p, T_i;
			subgraph.T_[index - 1].InvOut(T_i);
			iter->trans_.Output(T_p);
			iter->trans_.Set(T_i, index, toward);
			iter->trans_.Input(T_p);
		}
		else {
			Matrix4d T;
			subgraph.T_[toward - 1].Output(T);
			iter->trans_.Input(T);
		}
		iter++;
	}
}

void RemoveEdgeUsingPCInlier(vector<Geom>& shard, RankingSubgraph& graph)
{
	int num_shard = shard.size();
	list<LCSIndex>::iterator lcs_iter = graph.lcs_reference_.begin();

	for (; lcs_iter != graph.lcs_reference_.end(); ) {
		int c_node = lcs_iter->shard_x_;
		if (graph.node_[c_node - 1])
			c_node = lcs_iter->shard_y_;

		Matrix3d R_p, R_pi;
		Vector3d t_p, t_pi;
		if (lcs_iter->trans_.index_ == c_node) {
			lcs_iter->trans_.Output(R_p, t_p);
			lcs_iter->trans_.InvOut(R_pi, t_pi);
		}
		else {
			lcs_iter->trans_.InvOut(R_p, t_p);
			lcs_iter->trans_.Output(R_pi, t_pi);
		}

		shard[c_node - 1].MoveWOSurface(R_p, t_p);

		bool overlap = false;
		//########## Remove Overlap fragment
		for (int j = 0; j < num_shard; j++) {
			if (graph.node_[j]) {
				double area, size;

				overlap = OverlapCheck_3d(shard[j].edge_line_, 
								shard[c_node - 1].edge_line_,
								area, 
								size,
								100);

				if (overlap) {
					lcs_iter = graph.lcs_reference_.erase(lcs_iter);
					break;
				}
			}
		}

		if (!overlap) {
			bool pc_out = CountPCInlier(lcs_iter->inliner_, graph.node_, shard, c_node, 5.0, 3.0, false);
			if (pc_out)
				lcs_iter++;
			else {
				lcs_iter = graph.lcs_reference_.erase(lcs_iter);
			}
		}

		shard[c_node - 1].MoveWOSurface(R_pi, t_pi);
	}

	for (int j = 0; j < num_shard; j++) {
		if (graph.node_[j]) {
			Matrix3d R;
			Vector3d t;
			graph.T_[j].InvOut(R, t);
			shard[j].Move(R, t);
		}
	}
}

void RemoveSameGraph(vector<RankingSubgraph>& g_in, vector<RankingSubgraph>& g_out)
{
	g_out.clear();
	int num = g_in.size();
	bool* index = new bool[num];
	std::fill_n(index, num, false);

	for (int i = 0; i < num; i++) {
		if (index[i])
			continue;
		for (int j = i + 1; j < num; j++) {
			if (index[j])
				continue;
			else {
				if (g_in[i].isSimilarGraph(g_in[j]))
				{
					if (g_in[i].edge_.size() >= g_in[j].edge_.size()) {
						index[j] = true;
					}
					else {
						index[i] = true;
						break;
					}
				}
			}
		}
	}
	for (int i = 0; i < num; i++) {
		if (!index[i]) {
			g_out.push_back(g_in[i]);
		}
	}

	delete[] index;
}

void RemoveSameGraph(vector<RankingSubgraph>& g)
{
	vector<RankingSubgraph> tmp_g = g;
	g.clear();
	int num = tmp_g.size();
	bool* index = new bool[num];
	std::fill_n(index, num, false);

	for (int i = 0; i < num; i++) {
		if (index[i])
			continue;
		for (int j = i + 1; j < num; j++) {
			if (index[j])
				continue;
			else {
				if (tmp_g[i].isSimilarGraph(tmp_g[j]))
				{
					if (tmp_g[i].edge_.size() >= tmp_g[j].edge_.size()) {
						index[j] = true;
					}
					else {
						index[i] = true;
						break;
					}
				}
			}
		}
	}
	for (int i = 0; i < num; i++) {
		if (!index[i]) {
			g.push_back(tmp_g[i]);
		}
	}

	delete[] index;
}

void RemoveSameState(vector<State>& state)
{
	vector<State> tmp_s = state;
	state.clear();
	vector<State>().swap(state);

	int num = tmp_s.size();
	bool* index = new bool[num];
	std::fill_n(index, num, false);

	for (int i = 0; i < num; i++) {
		if (index[i])
			continue;
		for (int j = i + 1; j < num; j++) {
			if (index[j])
				continue;
			else {
				if (tmp_s[i].isSimilarState(tmp_s[j])) {
					if (tmp_s[i].state_score_ >= tmp_s[j].state_score_) {
						index[j] = true;
					}
					else {
						index[i] = true;
						break;
					}
				}
			}
		}
	}
	for (int i = 0; i < num; i++) {
		if (!index[i]) {
			state.push_back(tmp_s[i]);
		}
	}

	delete[] index;
}

bool isSameGraph(RankingSubgraph& g_a, RankingSubgraph& g_b)
{
	int num_node = g_a.node_.size();
	int num_edge = g_a.edge_.size();
	if (num_node != g_b.node_.size()) return false;
	else if (num_edge != g_b.edge_.size()) return false;

	// Investigate node first
	for (int i = 0; i < num_node; i++) {
		if (g_a.node_[i] != g_b.node_[i]) {
			return false;
		}
	}

	// Investigate edges 
	list<LCSIndex>::iterator a_iter = g_a.edge_.begin();
	list<LCSIndex>::iterator b_iter = g_b.edge_.begin();
	for (; a_iter != g_a.edge_.end(); ) {
		if (a_iter->index_ != b_iter->index_) {
			return false;
		}
		else {
			a_iter++;
			b_iter++;
		}
	}

	return true;
}

void PrepareGraphBuilding(vector<Geom>& shard,
	RankingSubgraph& toprank_graph)
{
	int num_shard = shard.size();

	//########## Find current reconstructing node from priority list
	int current_node = toprank_graph.NodeOut() - 1;
	toprank_graph.node_[current_node] = true;
	toprank_graph.node_log_.push_back(current_node + 1);
	toprank_graph.MakeTransLog();

	//########## Find edge from prioirty list
	vector<LCSIndex> edges = toprank_graph.EdgeOut();
	int num_priority_edges = edges.size();
	toprank_graph.edge_size_.push_back(num_priority_edges);

	//########## Insert used edge to toprank_graph edge set for building history
	for (int i = 0; i < num_priority_edges; i++) {
		pair<int, int> edge_link;
		edge_link.first = edges[i].shard_x_;
		edge_link.second = edges[i].shard_y_;

		toprank_graph.edge_.push_back(edges[i]);
		toprank_graph.edge_log_.push_back(edge_link);
		toprank_graph.simple_graph_(edges[i].shard_x_ - 1, edges[i].shard_y_ - 1) = 1;
		toprank_graph.simple_graph_(edges[i].shard_y_ - 1, edges[i].shard_x_ - 1) = 1;
	}

	//########## Initialize transformation matrix for ICP
	for (int i = 0; i < num_shard; i++) {

		Matrix3d R_prev;
		Vector3d t_prev;
		if (toprank_graph.node_[i]) {
			toprank_graph.T_[i].Output(R_prev, t_prev);

			shard[i].Move(R_prev, t_prev);
		}
	}
	//########## Move current piece based on pairwise transformation matrix
	Matrix3d R_p;
	Vector3d t_p;

	//########## In this Algorithm, pieces are always axis aligned initial state
	TransAverage(current_node + 1, edges, R_p, t_p);

	//########## Make ICP initial condiiton
	shard[current_node].Move(R_p, t_p);
	toprank_graph.T_[current_node].Input(R_p, t_p);
	toprank_graph.InputTransLog(current_node + 1, R_p, t_p);

	//########## Initialize 'matched_index_' member.
	if (!toprank_graph.matched_index_.empty()) {
		for (int i = 0; i < toprank_graph.matched_index_.size(); i++) {
			toprank_graph.matched_index_[i].clear();
			vector<bool>().swap(toprank_graph.matched_index_[i]);
		}
		vector<vector<bool>>().swap(toprank_graph.matched_index_);
	}
	for (int i = 0; i < num_shard; i++) {
		vector<bool> m_index(shard[i].edge_line_.point_.cols(), false);
		toprank_graph.matched_index_.push_back(m_index);
	}
}

void GraphAxisRefinement(vector<Geom>& shard,
	RankingSubgraph& toprank_graph)
{
	int num_shard = shard.size();
	vector<Geom*> geom_ptr;
	for (int i = 0; i < num_shard; i++) {
		if (toprank_graph.node_[i]) {
			geom_ptr.push_back(&shard[i]);
		}
	}

	Matrix3d R_a;
	Vector3d t_a;

	//########## Axis refinement process
	shard[toprank_graph.root_node_ - 1].edge_line_.axis_point_[0] = { 0, 0, 0 };
	shard[toprank_graph.root_node_ - 1].edge_line_.axis_norm_[0] = { 0, 0, 1 };
	RefineAxis(geom_ptr,
		shard[toprank_graph.root_node_ - 1].edge_line_.axis_point_[0],
		shard[toprank_graph.root_node_ - 1].edge_line_.axis_norm_[0],
		100, NUMBER_OF_THREAD, 0.5, true);
	//########## End axis refinement process

	AxisAlignment(shard[toprank_graph.root_node_ - 1].edge_line_, R_a, t_a);
	shard[toprank_graph.root_node_ - 1].SurMove(R_a, t_a);

	toprank_graph.T_[toprank_graph.root_node_ - 1].Input(R_a, t_a);
	toprank_graph.InputTransLog(toprank_graph.root_node_, R_a, t_a);
	for (int i = 0; i < num_shard; i++) {
		if ((i + 1 != toprank_graph.root_node_) && (toprank_graph.node_[i])) {
			shard[i].Move(R_a, t_a);
			toprank_graph.T_[i].Input(R_a, t_a);
			toprank_graph.InputTransLog(i + 1, R_a, t_a);
		}
	}
}

bool CheckGraphPlausibility(vector<Geom>& shard,
	RankingSubgraph& toprank_graph,
	string& log_path,
	int step_counter,
	int ext_index)
{
	bool return_value = true;
	int num_shard = shard.size();
	//################### First, Cehck score which is the number of inliers #################// 	
	if (toprank_graph.graph_score_ < 0)
	{
		return_value = false;
	}

	//################### Second, Check all pair-wise overlap #################// 
	if (return_value)
	{
		for (int i = 0; i < num_shard; i++) {
			for (int j = i + 1; j < num_shard; j++) {
				if ((toprank_graph.node_[i]) && (toprank_graph.node_[j])) {
					double area, size;
					bool overlap = OverlapCheck_3d(shard[i].edge_line_,
						shard[j].edge_line_,
						area,
						size,
						40);
					toprank_graph.max_overlap_area_ = std::max(area, toprank_graph.max_overlap_area_);
					if (overlap) {
						return_value = false;
						break;
					}
				}
			}
			if (!return_value) {
				break;
			}
		}
	}

	//################### Third, Check shape which is profile curves and rim condition #################// 
	vector<Vector3d> profile;
	if (return_value) {
		bool profile_matched = true;
		if (step_counter > 0) {
			bool interpolation = false, is_rim = false;
			double H_rim(0), R_rim(0);
			vector<Vector3d> rim_data;
			for (int i = 0; i < num_shard; i++) {
				if ((toprank_graph.node_[i])) {
					ToCylindricalInterpolation(shard[i].sur_in_, profile, interpolation);
					if (shard[i].edge_line_.is_seg_rim_) {
						is_rim = true;
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
			}
			double pc_var_value(0);
			if (!profile.empty()) {
				profile_matched = ProfileChecking(profile, 5.0, 7);
			}
			if (profile_matched && is_rim) {
				int up_count(0), down_count(0);
				MakeRadiusHeight(R_rim, H_rim, rim_data);
				MinMaxValue z_total;
				z_total.max_value = profile.back()(2);
				z_total.min_value = profile.front()(2);
				double top_gap = z_total.max_value - H_rim - 10;
				double bottom_gap = H_rim - z_total.min_value - 10;
				if ((top_gap > 0) && (bottom_gap > 0)) {
					profile_matched = false;
				}
			}
		}


		if (!profile_matched) {
			return_value = false;
		}
	}

	return return_value;
}