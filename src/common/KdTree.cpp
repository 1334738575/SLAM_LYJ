#include "KdTree.h"


NSP_SLAM_LYJ_MATH_BEGIN


KdTree2d::KdTree2d() {
	root = nullptr;
}
KdTree2d::~KdTree2d() {
	delete root;
}

bool KdTree2d::build(std::vector<Eigen::Vector2d>& _points) {
	if (root)
		delete root;
	if (_points.empty())
		return false;
	root = new Node{};
	std::vector<Eigen::Vector2d*> data(_points.size());
	for (size_t i = 0; i < _points.size(); i++)
	{
		data[i] = &(_points[i]);
	}
	root->build(data);
	return true;
}

int KdTree2d::knnSearch(const Eigen::Vector2d& _p, const int _size,
	std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists) {
	return search(_p, _size, -1, _points, _dists);
}
int KdTree2d::radiusSearch(const Eigen::Vector2d& _p, const double _r,
	std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists) {
	return search(_p, -1, _r, _points, _dists);
}
int KdTree2d::search(const Eigen::Vector2d& _p, const int _size, const double _r,
	std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists) {
	if(root == nullptr)
		return 0;
	_points.reserve(_size);
	_dists.reserve(_size);
	double r = _r;
	if (r < 0)
		r = DBL_MAX;
	//TODO
	std::stack<Node*> findNodes;
	root->search2(_p, findNodes);
	double comR = DBL_MAX;
	std::vector<std::pair<double, Eigen::Vector2d>> rets;
	while (!findNodes.empty()) {

		auto& node = findNodes.top();
		findNodes.pop();

		double d = node->dist(_p);
		if (d < r) {
			rets.emplace_back(d, node->data);
			//_points.push_back(node->data);
			//_dists.push_back(d);
		}
		if (d > comR)
			continue;
		comR = d;
		if (!findNodes.empty() && node->interest(_p, comR)) {
			auto& parentNode = findNodes.top();
			if(_p[parentNode->divideDim] >= parentNode->data[parentNode->divideDim] && parentNode->leftNode)
				parentNode->leftNode->search2(_p, findNodes);
			else if(_p[parentNode->divideDim] <= parentNode->data[parentNode->divideDim] && parentNode->rightNode)
				parentNode->rightNode->search2(_p, findNodes);
		}
	}
	int ss = (int)rets.size();
	if (ss > _size) {
		std::sort(rets.begin(), rets.end(), Node::compareSearch);
	}
	int outSize = ss > _size ? _size : ss;
	for (int i = 0; i < outSize;++i) {
		_points.push_back(rets[i].second);
		_dists.push_back(rets[i].first);
	}
	return (int)_points.size();


	//double squareR = _r * _r;
	//std::vector<std::pair<double, Eigen::Vector2d>> rets;
	//root->search(_p, _size, squareR, rets);
	//for (auto& ret : rets) {
	//	_points.push_back(ret.second);
	//}
	//return (int)_points.size();
}



NSP_SLAM_LYJ_MATH_END