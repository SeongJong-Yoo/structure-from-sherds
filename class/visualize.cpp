#include "Visualize.h"
#include "../class/ranking_system.h"
//############################## class visualize ##############################//
void Visualize::MakePointCloud(const MatrixXd& data, const MatrixXd& norm, string name)
{
	name_ = name;
	norm_name_ = name + "_n";
	data_ = MatrixtoCloud(data);
	norm_ = MatrixtoCloudNorm(norm);
}


void Visualize::MakeMesh(string& objFilePath, string name)
{
	mesh_name_ = name;
	pcl::io::loadPolygonFileOBJ(objFilePath, mesh_);
	pcl::fromPCLPointCloud2(mesh_.cloud, mesh_cloud_);
}

void Visualize::MakeMesh(pcl::PolygonMesh& input, string name)
{
	this->mesh_name_ = name;
	this->mesh_ = input;
	pcl::fromPCLPointCloud2(mesh_.cloud, mesh_cloud_);
}

void Visualize::AddPointCloud(view& viewer, int r, int g, int b, int size)
{
	double value = 0;
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, value, name_);
	if (value == 0) {
		if (!viewer->contains(name_)) {
			r_ = r, g_ = g, b_ = b;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color(data_, r_, g_, b_);
			viewer->addPointCloud<pcl::PointXYZ>(data_, Color, name_);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name_);
		}
		else viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, name_);
	}

	else if (value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, name_);
}

void Visualize::AddMesh(view& viewer)
{
	double value = 0;
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, value, mesh_name_);
	if (value == 0) {
		if (!viewer->contains(mesh_name_)) {
			viewer->addPolygonMesh(mesh_, mesh_name_, 0);
		}
		else viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, mesh_name_);
	}
	else if (value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, mesh_name_);
}

void Visualize::AddNormal(view& viewer, int level, int scale)
{
	double value = 0;
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, value, norm_name_);
	if (value == 0) {
		if (!viewer->contains(norm_name_)) {
			viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(data_, norm_, level, scale, norm_name_, 0);
		}
		else viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, norm_name_);
	}
	else if (value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, norm_name_);
}

void Visualize::UpdateData(view& viewer, const MatrixXd& data, const MatrixXd& norm, int r, int g, int b)
{
	MakePointCloud(data, norm, this->name_);
	r_ = r, g_ = g, b_ = b;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color(data_, r_, g_, b_);
	viewer->updatePointCloud(data_, Color, name_);
	viewer->removePointCloud(norm_name_, 0);
	viewer->spinOnce();
}

void Visualize::UpdateMesh(view& viewer, const pcl::PolygonMesh& mesh)
{
	mesh_ = mesh;
	viewer->updatePolygonMesh(mesh_, mesh_name_);
	viewer->spinOnce();
}


void Visualize::MeshTransform(const Matrix4d& trans, view& viewer)
{
	pcl::transformPointCloud(mesh_cloud_, mesh_cloud_, trans);
	pcl::toPCLPointCloud2(mesh_cloud_, mesh_.cloud);
	viewer->updatePolygonMesh(mesh_, mesh_name_);
}

void Visualize::MeshTransform(const Matrix3d& R, const Vector3d& t, view& viewer)
{
	Matrix4d trans = Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		trans.row(i) << R(i, 0), R(i, 1), R(i, 2), t[i];
	}
	this->MeshTransform(trans, viewer);
}

void Visualize::DataTransform(const Matrix4d& trans, view& viewer)
{
	pcl::transformPointCloud(*data_, *data_, trans);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color(data_, r_, g_, b_);
	viewer->updatePointCloud(data_, Color, name_);
}

void Visualize::DataTransform(const Matrix3d& R, const Vector3d& t, view& viewer)
{
	Matrix4d trans = Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		trans.row(i) << R(i, 0), R(i, 1), R(i, 2), t[i];
	}
	this->DataTransform(trans, viewer);
}

void Visualize::Transform(const Matrix4d& trans, view& viewer)
{
	DataTransform(trans, viewer);
	MeshTransform(trans, viewer);
}

void Visualize::Transform(const Matrix3d& R, const Vector3d& t, view& viewer)
{
	Matrix4d trans = Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		trans.row(i) << R(i, 0), R(i, 1), R(i, 2), t[i];
	}
	this->Transform(trans, viewer);
}

void Visualize::RemovePoint(view& viewer)
{
	viewer->removePointCloud(name_);
}

void Visualize::RemoveMesh(view& viewer)
{
	viewer->removePolygonMesh(mesh_name_);
}

void Visualize::TurnOffData(view& viewer)
{
	double value(0), n_value(0), m_value(0);
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, value, name_);
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, n_value, norm_name_);
	viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, m_value, mesh_name_);

	
	if (value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, name_);
	if (n_value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, norm_name_);
	if (m_value == 1) viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0, mesh_name_);
}

void Visualize::OutData(ptr_xyz& out_data, ptr_norm& out_norm)
{
	out_data = this->data_;
	out_norm = this->norm_;
}

void Visualize::OutMesh(pcl::PolygonMesh& out_mesh)
{
	out_mesh = this->mesh_;
}

void Visualize::SaveMesh(const string& path) const
{
	pcl::io::saveOBJFile(path, mesh_);
}

//############################## class VisSwitchVariables ##############################//
void VisSwitchVariables::KeyEvent(const string& str, const bool keydown)
{
	if (str == "space" && keydown) {
		first_ = true;
	}
	else if (str == "s" && keydown) {
		save_ = true;
	}
	else if (str == "o" && keydown)
	{
		obj_ = true;
	}
	else if (str == "Right" && keydown) {
		right_ = true;
	}
	else if (str == "Left" && keydown) {
		left_ = true;
	}
}

//############################## class cloud_corres ##############################//
void CloudCorres::VisualizeCor(string name, view& viewer)
{
	int num = pc_A_->points.size();
	if (!showing_) {
		if (name_.size() > 0) {
			for (int i = 0; i < num; i++) {
				viewer->setPointCloudRenderingProperties(1, 1, 1, name_[i]);
			}
		}

		else {
			for (int i = 0; i < num; i++) {
				string name_cor = name + std::to_string(i);
				viewer->addLine(pc_A_->points[i], pc_B_->points[i], name_cor);
				name_.push_back(name_cor);
			}
		}
		showing_ = true;
	}
	else {
		for (int i = 0; i < num; i++) {
			viewer->setPointCloudRenderingProperties(1, 0, 0, name_[i]);
		}
		showing_ = false;
	}
}

void CloudCorres::Update(view& viewer, ptr_xyz P_A, ptr_xyz P_B)
{
	string name = name_[0];
	pc_A_ = P_A;
	pc_B_ = P_B;
	Remove(viewer);
	VisualizeCor(name, viewer);
}

void CloudCorres::Remove(view& viewer)
{
	for (int i = 0; i < pc_A_->points.size(); i++) {
		viewer->removeShape(name_[i]);
	}
	showing_ = false;
	name_.clear();
}

//############################## other function ##############################//
ptr_xyz MatrixtoCloud(const MatrixXd& p)
{
	ptr_xyz Cloud(new pc_xyz);

	if (p.rows() != 3 && p.cols() == 3) {
		for (int i = 0; i < p.rows(); i++)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = p(i, 0);
			basic_point.y = p(i, 1);
			basic_point.z = p(i, 2);
			Cloud->points.push_back(basic_point);
		}
		Cloud->width = (int)Cloud->points.size();
		Cloud->height = 1;
	}

	else if (p.cols() != 3 && p.rows() == 3) {
		for (int i = 0; i < p.cols(); i++)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = p(0, i);
			basic_point.y = p(1, i);
			basic_point.z = p(2, i);
			Cloud->points.push_back(basic_point);
		}
		Cloud->width = (int)Cloud->points.size();
		Cloud->height = 1;
	}

	else cout << "MatrixtoCloud : Size error" << endl;

	return Cloud;
}

ptr_norm MatrixtoCloudNorm(const MatrixXd& p)
{
	ptr_norm Cloud(new pcl::PointCloud<pcl::Normal>);

	if (p.rows() != 3 && p.cols() == 3) {
		for (int i = 0; i < p.rows(); i++)
		{
			pcl::Normal basic_point;
			Cloud->back().normal_x = p(i, 0);
			Cloud->back().normal_y = p(i, 1);
			Cloud->back().normal_z = p(i, 2);
		}
	}

	else if (p.cols() != 3 && p.rows() == 3) {
		for (int i = 0; i < p.cols(); i++)
		{
			pcl::Normal basic_point;
			basic_point.normal_x = p(0, i);
			basic_point.normal_y = p(1, i);
			basic_point.normal_z = p(2, i);
			Cloud->push_back(basic_point);
		}
	}

	return Cloud;
}

CloudCorres CorresToCloud(const Corres& cor)
{
	ptr_xyz p_A(new pc_xyz), p_B(new pc_xyz);

	for (int i = 0; i < cor.cor.size(); i++) {
		pcl::PointXYZ basic_p_A, basic_p_B;
		basic_p_A.x = cor.cor[i].p_A(0);
		basic_p_A.y = cor.cor[i].p_A(1);
		basic_p_A.z = cor.cor[i].p_A(2);
		basic_p_B.x = cor.cor[i].p_B(0);
		basic_p_B.y = cor.cor[i].p_B(1);
		basic_p_B.z = cor.cor[i].p_B(2);

		p_A->points.push_back(basic_p_A);
		p_B->points.push_back(basic_p_B);
	}
	p_A->width = (int)p_A->points.size();
	p_A->height = 1;
	p_B->width = (int)p_B->points.size();
	p_B->height = 1;

	CloudCorres pc_cor(p_A, p_B);
	return pc_cor;

}


void VisCurrentState(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index,
	vector<Visualize>& pc_overlap,
	const vector<Trans>& T_axis)
{
	string logpath = manager.log_path_ + "/RESULT";
	int num_shard = pc.size();
	int num_total = manager.out_state_.size();
	int num_graph = manager.out_state_[index].graph_.size();
	cout << "Result : " << index + 1 << "/" << num_total << ", Score : "
		<< manager.out_state_[index].state_score_ << endl;

	//########## Save reconstruction result as transformation matrix ##########//
	//########## Path : ../ICCV Data/Graph Log/
	manager.out_state_[index].SaveLog(logpath, index, T_axis);

	for (int i = 0; i < num_graph; i++) {
		for (int j = 0; j < num_shard; j++) {
			if (manager.out_state_[index].graph_[i].node_[j]) {
				Matrix3d R; Vector3d t;
				manager.out_state_[index].graph_[i].T_[j].Output(R, t);
				t[1] = t[1] + 200 * i;
				EdgeLineMove(manager.shard_[j].edge_line_, R, t);

				pc[j].UpdateData(viewer, manager.shard_[j].edge_line_.point_, manager.shard_[j].edge_line_.normal_);
				pc[j].AddPointCloud(viewer);
				pc[j].MeshTransform(R, t, viewer);
			}
		}

		vector<Vector3d> overlap_points;
		for (int j = 0; j < num_shard; j++) {
			for (int k = j + 1; k < num_shard; k++) {
				if (manager.out_state_[index].graph_[i].node_[j] && manager.out_state_[index].graph_[i].node_[k]) {
					double area, size;
					bool overlap = OverlapCheck_3d(manager.shard_[j].edge_line_,
						manager.shard_[k].edge_line_,
						area,
						size,
						50,
						overlap_points);
				}
			}
		}
		MatrixXd pc_overlap_pair(3, overlap_points.size());
		for (int j = 0; j < overlap_points.size(); j++) {
			pc_overlap_pair.col(j) = overlap_points[j];
		}
		string name = "Instrusion_" + to_string(index);
		pc_overlap[index].MakePointCloud(pc_overlap_pair, pc_overlap_pair, name);

		cout << "Nodes" << endl;
		for (int j = 0; j < manager.out_state_[index].graph_[i].node_log_.size() - 1; j++) {
			cout << manager.out_state_[index].graph_[i].node_log_[j] << "->";
		}
		cout << manager.out_state_[index].graph_[i].node_log_.back() << endl;
		int edge_size = manager.out_state_[index].graph_[i].edge_log_.size();
		cout << "Edges : " << edge_size << endl;
		for (int j = 0; j < edge_size; j++) {
			int s_x = manager.out_state_[index].graph_[i].edge_log_[j].first;
			int s_y = manager.out_state_[index].graph_[i].edge_log_[j].second;
			cout << "(" << s_x << "-" << s_y << "), ";
		}
		cout << endl;
	}
	cout << "Score : " << manager.out_state_[index].state_score_ << endl;
}


void TurnoffandReturn(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index)
{
	int num_graph = manager.out_state_[index].graph_.size();
	int num_shard = pc.size();
	for (int i = 0; i < num_shard; i++) {
		pc[i].TurnOffData(viewer);
	}

	for (int i = 0; i < num_graph; i++) {
		for (int j = 0; j < num_shard; j++) {
			if (manager.out_state_[index].graph_[i].node_[j]) {
				Matrix3d R, R_i; Vector3d t, t_i;
				manager.out_state_[index].graph_[i].T_[j].Output(R, t);
				t[1] = t[1] + 200 * i;
				R_i = R.inverse();
				t_i = -R_i * t;
				EdgeLineMove(manager.shard_[j].edge_line_, R_i, t_i);

				pc[j].MeshTransform(R_i, t_i, viewer);
			}
		}
	}
}

void ObjVisualize(pcl::visualization::PCLVisualizer::Ptr& viewer,
	vector<Visualize>& pc,
	StateManager& manager,
	int index)
{
	int shard_num = manager.shard_.size();
	for (int i = 0; i < shard_num; i++) {
		if (manager.out_state_[index].true_node_[i]) {
			pc[i].AddMesh(viewer);
		}
	}
}

void SaveResult(const vector<Visualize>& pc,
	const StateManager& manager,
	int index,
	const string path)
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

	int num_shard = manager.shard_.size();
	string com_path = path + to_string(year) + "_" + to_string(mon) + "_"
		+ to_string(day) + "_" + to_string(hour) + "_" + to_string(min)
		+ "_Top_" + to_string(index + 1) + "_";
	for (int i = 0; i < num_shard; i++) {
		if (manager.out_state_[index].true_node_[i]) {
			string path_edge_line = com_path + "Edgeline_" + to_string(i + 1) + ".xyz";
			string path_obj = com_path + "OBJ_" + to_string(i + 1) + ".obj";
			pcl::PolygonMesh save_mesh;
			SaveEdgeLine(manager.shard_[i].edge_line_, path_edge_line);
			pc[i].SaveMesh(path_obj);
		}
	}
}