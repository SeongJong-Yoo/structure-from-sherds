#include "data_structure.h"

//############################## class BreakLine ##############################//
BreakLine::BreakLine() {
	is_seg_rim_ = false;
	is_seg_base_ = false;
	is_sane_base_ = false;
	is_moving_ = false;
	is_inverted_order_ = false;
	is_tree_ = false;
	tree_ = nullptr;
}

BreakLine::~BreakLine() {
	if (is_tree_) {
		RemoveTree();
	}
}

BreakLine::BreakLine(const BreakLine& rhs)
	: tree_(nullptr), is_tree_(false)
{
	point_ = rhs.point_;
	normal_ = rhs.normal_;
	line_normal_ = rhs.line_normal_;
	feature_ = rhs.feature_;
	index_ = rhs.index_;
	axis_point_ = rhs.axis_point_;
	axis_norm_ = rhs.axis_norm_;
	point_index_ = rhs.point_index_;
	is_seg_rim_ = rhs.is_seg_rim_;
	is_seg_base_ = rhs.is_seg_base_;
	is_sane_base_ = rhs.is_sane_base_;
	is_moving_ = rhs.is_moving_;
	is_inverted_order_ = rhs.is_inverted_order_;
	if (rhs.is_tree_) {
		BuildTree();
	}
}

BreakLine::BreakLine(BreakLine&& rhs)
{
	point_ = move(rhs.point_);
	normal_ = move(rhs.normal_);
	line_normal_ = move(rhs.line_normal_);
	feature_ = move(rhs.feature_);
	index_ = move(rhs.index_);
	axis_point_ = move(rhs.axis_point_);
	axis_norm_ = move(rhs.axis_norm_);
	point_index_ = move(rhs.point_index_);
	is_seg_rim_ = rhs.is_seg_rim_;
	is_seg_base_ = rhs.is_seg_base_;
	is_sane_base_ = rhs.is_sane_base_;
	is_moving_ = rhs.is_moving_;
	is_inverted_order_ = rhs.is_inverted_order_;

	tree_ = rhs.tree_;
	is_tree_ = true;
	rhs.tree_ = nullptr;
	rhs.is_tree_ = false;
}

BreakLine& BreakLine::operator=(const BreakLine& rhs)
{
	point_ = rhs.point_;
	normal_ = rhs.normal_;
	line_normal_ = rhs.line_normal_;
	feature_ = rhs.feature_;
	index_ = rhs.index_;
	axis_point_ = rhs.axis_point_;
	axis_norm_ = rhs.axis_norm_;
	point_index_ = rhs.point_index_;
	is_seg_rim_ = rhs.is_seg_rim_;
	is_seg_base_ = rhs.is_seg_base_;
	is_sane_base_ = rhs.is_sane_base_;
	is_moving_ = rhs.is_moving_;
	is_inverted_order_ = rhs.is_inverted_order_;
	if (is_tree_) {
		RemoveTree();
	}
	if (rhs.is_tree_) {
		BuildTree();
	}
	return *this;
}

BreakLine& BreakLine::operator=(BreakLine&& rhs)
{
	point_ = move(rhs.point_);
	normal_ = move(rhs.normal_);
	line_normal_ = move(rhs.line_normal_);
	feature_ = move(rhs.feature_);
	index_ = move(rhs.index_);
	axis_point_ = move(rhs.axis_point_);
	axis_norm_ = move(rhs.axis_norm_);
	point_index_ = move(rhs.point_index_);
	is_seg_rim_ = rhs.is_seg_rim_;
	is_seg_base_ = rhs.is_seg_base_;
	is_sane_base_ = rhs.is_sane_base_;
	is_moving_ = rhs.is_moving_;
	is_inverted_order_ = rhs.is_inverted_order_;
	if (is_tree_) {
		RemoveTree();
	}
	tree_ = rhs.tree_;
	is_tree_ = true;
	rhs.tree_ = nullptr;
	rhs.is_tree_ = false;
	return *this;
}

void BreakLine::BuildTree(void)
{
	tree_ = kd_create(3);
	int num_index = this->index_.cols();

	for (int i = 0; i < point_.cols(); i++) {
		if (this->is_seg_rim_) {
			for (int j = 0; j < num_index; j++) {
				if ((i + 1 > this->index_(0, j)) && (i + 1 < this->index_(1, j))) {
					if (!this->index_(2, j)) {
						assert(kd_insert(tree_, &point_(0, i), &point_index_[i]) == 0);
						if (kd_insert(tree_, &point_(0, i), &point_index_[i]) != 0) cout << "build tree error" << endl;
						break;
					}
				}
			}
		}
		else {
			assert(kd_insert(tree_, &point_(0, i), &point_index_[i]) == 0);
			if (kd_insert(tree_, &point_(0, i), &point_index_[i]) != 0) cout << "build tree error" << endl;
		}
		
	}
}

void BreakLine::RemoveTree(void)
{
	kd_free(tree_);
}

void BreakLine::ReadAxis(const string & file)
{
	ifstream fin;

	fin.open(file, ios::binary);

	double x, y, z, nx, ny, nz;

	while (true) {
		fin >> x;
		if (!fin.good()) {
			break;
		}

		fin >> y >> z >> nx >> ny >> nz;
		if (isnan(nx) || isnan(ny) || isnan(nz)) cerr << "BreakLine::read_data --> NAN" << endl;
		axis_point_.push_back(Vector3d(x, y, z));
		axis_norm_.push_back(Vector3d(nx, ny, nz));
	}

	fin.close();

	feature_.resize(axis_point_.size());
}

void BreakLine::ReadPCDFileWithInfo(const string& pcdFilePath)
{
	stringstream ss;
	ifstream myfile(pcdFilePath);
	std::vector<std::string> fileContents;
	std::string str;
	std::string file_contents;
	int numberOfPoints;

	int totalSegements = 0, totalPts = 0;
	is_seg_rim_ = false;

	//Reading file contents
	while (std::getline(myfile, str))
	{
		fileContents.push_back(str);
	}

	std::vector<std::string> results;
	boost::algorithm::split(results, fileContents[1], [](char c) {return c == ' '; });


	if (results[0] == "#")
	{
		totalSegements = std::stoi(results[1]);
		totalPts = std::stoi(results[2]);
		index_.resize(3, totalSegements);
		int info_index = stoi(results[3]);
		is_seg_rim_ = (abs(info_index) == 1 || abs(info_index) == 3) ? true : false;
		is_seg_base_ = (abs(info_index) == 2 || abs(info_index) == 3) ? true : false;
		is_sane_base_ = (is_seg_base_ && (info_index > 0)) ? true : false;
		for (size_t s = 0; s < totalSegements; s++)
		{
			boost::split(results, fileContents[s + 2], [](char c) {return c == ' '; });
			index_(0, s) = std::stoi(results[1]);
			index_(1, s) = std::stoi(results[2]);
			index_(2, s) = std::stoi(results[3]);
		}
	}
	else
	{
		cout << "Error reading information about segments and rim." << endl;
	}

	//removing all rows except the data
	fileContents.erase(fileContents.begin(), fileContents.begin() + totalSegements + 12);


	//Constructing the matrix
	numberOfPoints = fileContents.size();

	MatrixXd point_tmp(3, numberOfPoints);
	MatrixXd normal_tmp(3, numberOfPoints);

	for (int i = 0; i < numberOfPoints; i++) 
	{
		ss << fileContents[i];
		ss >> point_tmp(0, i);
		ss >> point_tmp(1, i);
		ss >> point_tmp(2, i);
		ss >> normal_tmp(0, i);
		ss >> normal_tmp(1, i);
		ss >> normal_tmp(2, i);
		ss.str(std::string());
	}
	vector<bool> single_point(numberOfPoints, false);
	int num_point(0);
	for (int i = 0; i < numberOfPoints; i++) {
		Vector3d point_prior, point;
		if (i == numberOfPoints - 1) {
			point_prior = point_tmp.col(0);
			point = point_tmp.col(numberOfPoints - 1);
		}
		else {
			point_prior = point_tmp.col(i + 1);
			point = point_tmp.col(i);
		}

		double size = (point_prior - point).norm();
		if (size > 0) {
			single_point[i] = true;
			num_point++;
		}
	}
	point_.resize(3, num_point);
	normal_.resize(3, num_point);
	feature_.resize(axis_point_.size());
	point_index_.resize(num_point);
	int counter(0);
	for (int i = 0; i < numberOfPoints; i++) {
		if (single_point[i]) {
			point_.col(counter) = point_tmp.col(i);
			normal_.col(counter) = normal_tmp.col(i);
			point_index_[counter] = counter;
			counter++;
		}
	}	
}

void BreakLine::CalculateLineNormal(void)
{
	int num_point = point_.cols();
	line_normal_.resize(3, num_point);
	for (int i = 0; i < num_point; i++) {
		Vector3d point_prior, point;
		if (i == num_point - 1) {
			point_prior = point_.col(0);
			point = point_.col(num_point - 1);
		}
		else {
			point_prior = point_.col(i + 1);
			point = point_.col(i);
		}
		Vector3d sur_normal = normal_.col(i);
		Vector3d line_normal = (point_prior - point).cross(sur_normal);
		line_normal_.col(i) = line_normal / line_normal.norm();
	}
}

void BreakLine::ChangeSequence(MatrixXd& src, int mode = 0)
{
	int Row = src.rows();
	int Col = src.cols();
	MatrixXd Inverse(Row, Col);

	if (Row > Col) {
		for (size_t i = 0; i < Row; i++) {
			for (size_t j = 0; j < Col; j++) {
				if (mode == 1) {
					if (i > 3) {
						Inverse(i, j) = src(Row - i - 1, j);
					}
					else {
						Inverse(i, j) = -1 * src(Row - i - 1, j);
					}
				}
				else Inverse(i, j) = src(Row - i - 1, j);
			}
		}
	}

	else {
		for (size_t i = 0; i < Col; i++) {
			for (size_t j = 0; j < Row; j++) {
				if (mode == 1) {
					if (j > 3) {
						Inverse(j, i) = src(j, Col - i - 1);
					}
					else {
						Inverse(j, i) = -1 * src(j, Col - i - 1);
					}
				}
				else Inverse(j, i) = src(j, Col - i - 1);
			}
		}
	}


	src = Inverse;
}

void BreakLine::ChangeOrder(void)
{
	int num = point_.cols();	// Changed : feature_.cols() -> point_.cols();
	int numseg = index_.cols();

	ChangeSequence(point_);
	ChangeSequence(normal_);
	ChangeSequence(line_normal_);
	for (int i = 0; i < feature_.size(); i++) {
		ChangeSequence(feature_[i], 1);
	}

	for (int i = 0; i < numseg; i++) {
		int tem(0);
		tem = num - index_(0, i) + 1;
		index_(0, i) = num - index_(1, i) + 1;
		index_(1, i) = tem;
	}

	for (int i = 0; i < num; i++) {
		point_index_[i] = num - point_index_[i] - 1;
	}
	if (this->is_inverted_order_)
		this->is_inverted_order_ = false;
	else if (!this->is_inverted_order_)
		this->is_inverted_order_ = true;
}

void BreakLine::Remove(void)
{
	point_.resize(3, 1);
	normal_.resize(3, 1);
	axis_norm_.clear();
	axis_point_.clear();
	feature_.clear();
	index_.resize(3, 1);

	point_.col(0) << 0, 0, 0;
	normal_.col(0) << 0, 0, 0;
	index_.col(0) << 0, 0, 0;
	is_seg_rim_ = false;
	is_moving_ = false;
}


//############################## class Geom ##############################//
void Geom::LoadSurface(const string& sur_in, const string& sur_out, int num_data)
{
	ReadXYZ(sur_in, sur_in_.point_, sur_in_.normal_, num_data);
	ReadXYZ(sur_out, sur_out_.point_, sur_out_.normal_, num_data);
	sur_in_.is_seg_base_ = sur_out_.is_seg_base_ = edge_line_.is_seg_base_;
	sur_in_.is_seg_rim_ = sur_out_.is_seg_rim_ = edge_line_.is_seg_rim_;
}

void Geom::SurMove(const Matrix3d& R, const Vector3d& t)
{
	EdgeLineMove(sur_in_, R, t);
	EdgeLineMove(sur_out_, R, t);
}

void Geom::AxisMove(const Matrix3d& R, const Vector3d& t)
{
	for (int i = 0; i < edge_line_.axis_point_.size(); i++) {
		edge_line_.axis_norm_[i] = R * edge_line_.axis_norm_[i];
		edge_line_.axis_point_[i] = R * edge_line_.axis_point_[i] + t;
	}
}

void Geom::Move(const Matrix3d& R, const Vector3d& t)
{
	SurMove(R, t);
	EdgeLineMove(edge_line_, R, t);
	AxisMove(R, t);
}

void Geom::MoveWOSurface(const Matrix3d& R, const Vector3d& t)
{
	EdgeLineMove(edge_line_, R, t);
	AxisMove(R, t);
}

void Geom::RemoveData(void)
{
	edge_line_.Remove();
	//sur_frac_.remove();
	sur_in_.Remove();
	sur_out_.Remove();
}

//############################## class Trans ##############################//
void Trans::Set(Matrix3d Ri, Vector3d ti, int index, int toward)
{
	T_ = Matrix4d::Identity();
	R_ = Ri;
	t_ = ti;
	index_ = index;
	toward_ = toward;
	for (int i = 0; i < 3; i++) {
		T_.row(i) << Ri(i, 0), Ri(i, 1), Ri(i, 2), ti[i];
	}
}

void Trans::Set(Matrix4d Ti, int index, int toward)
{
	T_ = Ti;
	index_ = index;
	toward_ = toward;
	for (int i = 0; i < 3; i++) {
		R_.row(i) << Ti(i, 0), Ti(i, 1), Ti(i, 2);
		t_[i] = Ti(i, 3);
	}
}

void Trans::Input(Matrix3d Ri, Vector3d ti)
{
	Matrix4d Ti = Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		Ti.row(i) << Ri(i, 0), Ri(i, 1), Ri(i, 2), ti[i];
	}
	T_ = Ti * T_;
	R_ = Ri * R_;
	t_ = Ri * t_ + ti;
}

void Trans::Input(Matrix4d Ti)
{
	Matrix3d Ri = Matrix3d::Identity();
	Vector3d ti = { 0, 0, 0 };
	for (int i = 0; i < 3; i++) {
		Ri.row(i) << Ti(i, 0), Ti(i, 1), Ti(i, 2);
		ti[i] = Ti(i, 3);
	}

	T_ = Ti * T_;
	R_ = Ri * R_;
	t_ = Ri * t_ + ti;
}

void Trans::Print()
{
	cout << "Transformation " << index_ << " to " << toward_ << endl;
	cout << T_ << endl;
}

void Trans::Save(const string& filePath)
{
	string Name = filePath +  "T_"  + std::to_string(index_) + "to_" + std::to_string(toward_) + ".txt";
	ofstream writeStream(Name, ios::out | ios::trunc);
	if (writeStream) {
		for (size_t i = 0; i < 4; i++) {
			for (size_t j = 0; j < 4; j++) {
				writeStream << T_(i, j) << " ";
			}
			writeStream << endl;
		}
		writeStream.close();
	}
	else
		cout << "Transformaton matrix file writes error" << endl;
}

void Trans::Output(Matrix4d& T_out) const
{
	T_out = T_;
}

void Trans::Output(Matrix3d& R_out, Vector3d& t_out) const
{
	R_out = R_;
	t_out = t_;
}


void Trans::InvOut(Matrix3d& R_out, Vector3d& t_out) const
{
	R_out = R_.inverse();
	t_out = -R_out * t_;
}

void Trans::InvOut(Matrix4d& T_out) const
{
	T_out = T_.inverse();
}

bool Trans::isSimilar(const Trans& comp, double rad_threshold, double t_threshold)
{
	Matrix4d T_c;
	Matrix3d R;
	Vector3d t, w;
	comp.InvOut(T_c);

	Matrix4d T = T_ * T_c;

	for (int i = 0; i < 3; i++) {
		R.row(i) << T(i, 0), T(i, 1), T(i, 2);
		t[i] = T(i, 3);
	}

	Matrix3d log_R = R.log();
	w << -log_R(0, 1), log_R(0, 2), -log_R(1, 2);

	double deg = w.norm();	// Radian
	double trans = t.norm();
	
	if ((deg < rad_threshold) && (trans < t_threshold)) return true;
	else return false;
}

bool Trans::isSimilar(const Matrix3d& R_c, double rad_threshold)
{
	Matrix3d R_out = R_ * R_c.transpose();
	Matrix3d log_R = R_out.log();
	Vector3d w;
	w << -log_R(1, 2), log_R(0, 2), -log_R(0, 1);

	double deg = w.norm();

	if (deg < rad_threshold) return true;
	else return false;
}

bool Trans::isSimilar(const Matrix3d& R_c, const Vector3d& t_c, double rad_threshold, double t_threshold)
{
	Matrix4d T_c = Matrix4d::Identity();

	for (int i = 0; i < 3; i++) {
		T_c.row(i) << R_c(i, 0), R_c(i, 1), R_c(i, 2), t_c[i];
	}
	return isSimilarTrans(T_, T_c, rad_threshold, t_threshold);
}
void Trans::Initialize(void)
{
	index_ = 0;
	toward_ = 0;
	R_ = Matrix3d::Identity();
	t_ = Vector3d(0, 0, 0);
	for (int i = 0; i < 3; i++) {
		T_.row(i) << R_(i, 0), R_(i, 1), R_(i, 2), t_[i];
	}
}

void Trans::Read(const string& path)
{
	stringstream ss;
	ifstream myfile(path);
	string str;
	vector<string> fileContents;
	int num;

	while (getline(myfile, str)) {
		fileContents.push_back(str);
	}
	num = fileContents.size();
	T_ = Matrix4d::Identity();
	R_ = Matrix3d::Identity();
	t_ = { 0, 0, 0 };
	for (int i = 0; i < num - 1; i++) {
		ss << fileContents[i];
		ss >> R_(i, 0);
		ss >> R_(i, 1);
		ss >> R_(i, 2);
		ss >> t_[i];
		ss.clear();
	}
	for (int i = 0; i < 3; i++) {
		T_.row(i) << R_(i, 0), R_(i, 1), R_(i, 2), t_[i];
	}
}

bool isSimilarTrans(const Matrix4d& T_a, const Matrix4d& T_b, const double& rad_threshold, const double& t_threshold)
{
	Matrix3d R;
	Vector3d t, w;
	Matrix4d T = T_a * T_b.inverse();

	for (int i = 0; i < 3; i++) {
		R.row(i) << T(i, 0), T(i, 1), T(i, 2);
		t[i] = T(i, 3);
	}

	Matrix3d log_R = R.log();
	w << -log_R(1, 2), log_R(0, 2), -log_R(0, 1);

	double deg = w.norm();	// Radian
	double trans = t.norm();

	if ((deg < rad_threshold) && (trans < t_threshold)) return true;
	else return false;
}

//############################## class LCSIndex ##############################//
bool LCSIndex::SamePart(LCSIndex& b)
{
	if (this->shard_x_ == b.shard_x_ && this->shard_y_ == b.shard_y_) {
		return true;
	}
	else if (this->shard_x_ == b.shard_y_ && this->shard_y_ == b.shard_x_) {
		return true;
	}
	else return false;
}

void LCSIndex::InvertOrder(int shard, int num)
{
	if (shard == shard_x_) {
		int tem(0);
		tem = num - start_.x - 1;
		start_.x = num - end_.x - 1;
		end_.x = tem;
	}
	else if (shard == shard_y_) {
		int tem(0);
		tem = num - start_.y - 1;
		start_.y = num - end_.y - 1;
		end_.y = tem;
	}
	else {
		cout << "LCS InvertOrder : Wrong shard input" << endl;
	}
}

//############################## other function ##############################//
void MatrixMove(MatrixXd& p, const MatrixXd& R, const Vector3d& t)
{
	omp_set_num_threads(NUMBER_OF_THREAD);
	#pragma omp parallel for
	for (int i = 0; i < p.cols(); i++) {
		p.col(i) = R * p.col(i) + t;
	}
}

void EdgeLineMove(BreakLine& L, const Matrix3d& R, const Vector3d& t)
{
	Vector3d zero = { 0, 0, 0 };
	MatrixMove(L.point_, R, t);
	MatrixMove(L.normal_, R, zero);
	MatrixMove(L.line_normal_, R, zero);
}

void ReadXYZ(const string& path, MatrixXd& point_, MatrixXd& normal_, int num_data)
{
	stringstream ss;
	ifstream myfile(path);
	string str;
	vector<string> fileContents;
	//int num;

	while (getline(myfile, str)) {
		fileContents.push_back(str);
	}
	if (num_data == 0) {
		num_data = fileContents.size();
	}
	int interval = (int)floor(fileContents.size() / num_data);
	point_.resize(3, num_data);
	normal_.resize(3, num_data);
	for (int i = 0; i < num_data; i++) {
		int point_index = interval * i;
		ss << fileContents[point_index];
		ss >> point_(0, i);
		ss >> point_(1, i);
		ss >> point_(2, i);
		ss >> normal_(0, i);
		ss >> normal_(1, i);
		ss >> normal_(2, i);
		ss.clear();
	}
}

void ReadPCD(const string& path,
	MatrixXd& point_,
	MatrixXd& normal_,
	int num_data)
{
	stringstream ss;
	ifstream myfile(path);
	std::vector<std::string> fileContents;
	std::string str;
	std::string file_contents;

	//Reading file contents
	while (std::getline(myfile, str))
	{
		fileContents.push_back(str);
	}
	//removing all rows except the data
	fileContents.erase(fileContents.begin(), fileContents.begin() + 12);

	if (num_data == 0) {
		num_data = fileContents.size();
	}

	int interval = (int)floor(fileContents.size() / num_data);
	point_.resize(3, num_data);
	normal_.resize(3, num_data);

	for (int i = 0; i < num_data; i++)
	{
		int point_index = interval * i;
		ss << fileContents[point_index];
		ss >> point_(0, i);
		ss >> point_(1, i);
		ss >> point_(2, i);
		ss >> normal_(0, i);
		ss >> normal_(1, i);
		ss >> normal_(2, i);
		ss.str(std::string());
	}
}

void SaveEdgeLine(const MatrixXd& src, const string& path)
{
	ofstream writeStream(path, ios::out | ios::trunc);

	size_t num_point = src.cols();
	if (writeStream) {
		for (size_t i = 0; i < num_point; i++) {
			writeStream << src(0, i) << " ";
			writeStream << src(1, i) << " ";
			writeStream << src(2, i) << " " << endl;
		}
		writeStream.close();
	}
	else {
		cout << "SaveBreakline : File opening error" << endl;
	}
}


void SaveEdgeLine(const vector<Vector3d>& src, const string& path)
{
	ofstream writeStream(path, ios::out | ios::trunc);

	size_t num_point = src.size();
	if (writeStream) {
		for (size_t i = 0; i < num_point; i++) {
			writeStream << src[i](0) << " ";
			writeStream << src[i](1) << " ";
			writeStream << src[i](2) << " " << endl;
		}
		writeStream.close();
	}
	else {
		cout << "SaveEdgeLine : File opening error" << endl;
	}
}

void SaveEdgeLine(const BreakLine& src, const string& path)
{
	ofstream writeStream(path, ios::out | ios::trunc);

	size_t num_point = src.point_.cols();
	if (writeStream) {
		for (size_t i = 0; i < num_point; i++) {
			writeStream << src.point_(0, i) << " ";
			writeStream << src.point_(1, i) << " ";
			writeStream << src.point_(2, i) << " ";
			writeStream << src.normal_(0, i) << " ";
			writeStream << src.normal_(1, i) << " ";
			writeStream << src.normal_(2, i) << " " << endl;
		}
		writeStream.close();
	}
	else {
		cout << "SaveBreakline : File opening error" << endl;
	}
}
