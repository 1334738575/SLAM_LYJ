#ifndef SLAM_LYJ_KDTREE_H
#define SLAM_LYJ_KDTREE_H

#include "base/Base.h"


NSP_SLAM_LYJ_MATH_BEGIN

template<typename T, int DIM>
class KdTree
{
public:
	using TmpVecX = Eigen::Matrix<T, DIM, 1>;
	using DataNode = std::pair<TmpVecX, int>;
	using DataIn = std::pair<TmpVecX*, int>;
	class Node
	{
	public:
		DataNode data;
		Node* leftNode = nullptr;
		Node* rightNode = nullptr;
		int divideDim = 0;

		static bool compareSearch(const std::pair<T, TmpVecX>& _v1, const std::pair<T, TmpVecX>& _v2) {
			if (_v1.first < _v2.first)
				return true;
			return false;
		}
		static bool compareSearch2(const std::tuple<T, TmpVecX, int>& _v1, const std::tuple<T, TmpVecX, int>& _v2) {
			if (std::get<0>(_v1) < std::get<0>(_v2))
				return true;
			return false;
		}

		void build(std::vector<DataIn> _points) {
			if (_points.size() == 1) {
				data.first = *(_points[0].first);
				data.second = _points[0].second;
				return;
			}
			int halfSize = (int)_points.size() / 2;
			int dim = divideDim;
			std::sort(_points.begin(), _points.end(), [dim](const DataIn& _p1, const DataIn& _p2)->bool {
				if ((*(_p1.first))[dim] < (*(_p2.first))[dim])
					return true;
				return false;
			});
			data.first = *(_points[halfSize].first);
			data.second = _points[halfSize].second;
			leftNode = new Node{};
			int nextDim = (divideDim + 1) % DIM;
			leftNode->divideDim = nextDim;
			leftNode->build(std::vector<DataIn>(_points.begin(), _points.begin() + halfSize));
			if (_points.size() > 2) {
				rightNode = new Node{};
				rightNode->divideDim = nextDim;
				rightNode->build(std::vector<DataIn>(_points.begin() + halfSize + 1, _points.end()));
			}
		}
		T dist(const TmpVecX& _p) {
			return (_p - data.first).norm();
		}
		bool interest(const TmpVecX& _p, T& _minD) {
			T d = std::abs(_p[divideDim] - data.first[divideDim]);
			if (d < _minD)
				return true;
			return false;
		}
		void search2(const TmpVecX& _p, std::stack<Node*>& _rets) {
			_rets.push(this);
			if (_p[divideDim] <= data.first[divideDim] && leftNode)
				leftNode->search2(_p, _rets);
			else if (_p[divideDim] >= data.first[divideDim] && rightNode)
				rightNode->search2(_p, _rets);
		}

		~Node() {
			if (leftNode)
				delete leftNode;
			if (rightNode)
				delete rightNode;
		}
	};

	KdTree() {}
	~KdTree() {}

	bool build(std::vector<TmpVecX>& _points, std::function<T(const Node*, const TmpVecX&)> _funcDist=nullptr) {
		if (root)
			delete root;
		if (_points.empty())
			return false;
		funcDist = _funcDist;
		root = new Node{};
		std::vector<DataIn> data(_points.size());
		for (size_t i = 0; i < _points.size(); i++) {
			data[i].first = &(_points[i]);
			data[i].second = (int)i;
		}
		root->build(data);
		return true;
	}
	bool setFuncDist(std::function<T(const Node*, const TmpVecX&)> _funcDist) {
		funcDist = _funcDist;
	}
	int knnSearch(const TmpVecX& _p, const int _size, std::vector<TmpVecX>& _points, std::vector<T>& _dists) {
		return search(_p, _size, static_cast<T>(-1), _points, _dists);
	}
	int radiusSearch(const TmpVecX& _p, const T _r, std::vector<TmpVecX>& _points, std::vector<T>& _dists) {
		return search(_p, -1, _r, _points, _dists);
	}
	int search(const TmpVecX& _p, const int _size, const T _r, std::vector<TmpVecX>& _points, std::vector<T>& _dists) {
		if (root == nullptr)
			return 0;
		_points.reserve(_size);
		_dists.reserve(_size);
		std::stack<Node*> findNodes;
		root->search2(_p, findNodes);
		T comR = root->dist(_p);
		std::vector<std::pair<T, TmpVecX>> rets;
		while (!findNodes.empty()) {
			auto& node = findNodes.top();
			findNodes.pop();
			T d;
			if (funcDist)
				d = funcDist(node, _p);
			else
				d = node->dist(_p);
			if (_r < 0 || d < _r)
				rets.emplace_back(d, node->data.first);
			if (d > comR)
				continue;
			comR = d;
			if (!findNodes.empty() && node->interest(_p, comR)) {
				auto& parentNode = findNodes.top();
				if (_p[parentNode->divideDim] >= parentNode->data.first[parentNode->divideDim] && parentNode->leftNode)
					parentNode->leftNode->search2(_p, findNodes);
				else if (_p[parentNode->divideDim] <= parentNode->data.first[parentNode->divideDim] && parentNode->rightNode)
					parentNode->rightNode->search2(_p, findNodes);
			}
		}
		int ss = (int)rets.size();
		if (ss > _size) {
			std::sort(rets.begin(), rets.end(), Node::compareSearch);
		}
		int outSize = ss > _size ? _size : ss;
		for (int i = 0; i < outSize; ++i) {
			_points.push_back(rets[i].second);
			_dists.push_back(rets[i].first);
		}
		return (int)_points.size();
	}
	int search2(const TmpVecX& _p, const int _size, const T _r,
		std::vector<std::pair<TmpVecX, int>>& _points, std::vector<T>& _dists) {
		if (root == nullptr)
			return 0;
		_points.reserve(_size);
		_dists.reserve(_size);
		std::stack<Node*> findNodes;
		root->search2(_p, findNodes);
		T comR = root->dist(_p);
		std::vector<std::tuple<T, TmpVecX, int>> rets;
		while (!findNodes.empty()) {
			auto& node = findNodes.top();
			findNodes.pop();
			T d;
			if (funcDist)
				d = funcDist(node, _p);
			else
				d = node->dist(_p);
			if (_r < 0 || d < _r)
				rets.emplace_back(d, node->data.first, node->data.second);
			if (d > comR)
				continue;
			comR = d;
			if (!findNodes.empty() && node->interest(_p, comR)) {
				auto& parentNode = findNodes.top();
				if (_p[parentNode->divideDim] >= parentNode->data.first[parentNode->divideDim] && parentNode->leftNode)
					parentNode->leftNode->search2(_p, findNodes);
				else if (_p[parentNode->divideDim] <= parentNode->data.first[parentNode->divideDim] && parentNode->rightNode)
					parentNode->rightNode->search2(_p, findNodes);
			}
		}
		int ss = (int)rets.size();
		if (ss > _size) {
			std::sort(rets.begin(), rets.end(), Node::compareSearch2);
		}
		int outSize = ss > _size ? _size : ss;
		for (int i = 0; i < outSize; ++i) {
			_points.emplace_back(std::get<1>(rets[i]), std::get<2>(rets[i]));
			_dists.push_back(std::get<0>(rets[i]));
		}
		return (int)_points.size();
	}

private:
	Node* root = nullptr;
	std::function<T(const Node*, const TmpVecX&)> funcDist = nullptr; //可以指定搜索时的距离判断方式（去除一定点或者添加额外判断）
};



class KdTree2d
{
public:

	class Node
	{
	public:
		Node() {};
		Eigen::Vector2d data;
		Node* leftNode = nullptr;
		Node* rightNode = nullptr;
		int divideDim = 0;

		static bool compare0(const Eigen::Vector2d* _v1, const Eigen::Vector2d* _v2) {
			if ((*_v1)[0] < (*_v2)[0])
				return true;
			return false;
		}
		static bool compare1(const Eigen::Vector2d* _v1, const Eigen::Vector2d* _v2) {
			if ((*_v1)[1] < (*_v2)[1])
				return true;
			return false;
		}
		static bool compareSearch(const std::pair<double, Eigen::Vector2d>& _v1, const std::pair<double, Eigen::Vector2d>& _v2) {
			if (_v1.first < _v2.first)
				return true;
			return false;
		}

		//输入有序数据
		void build(std::vector<Eigen::Vector2d*> _points) {
			if (_points.size() == 1) {
				data = *(_points[0]);
				return;
			}
			int ind = 0;
			int halfSize = (int)_points.size() / 2;
			if (divideDim == 0)
				std::sort(_points.begin(), _points.end(), compare0);
			else
				std::sort(_points.begin(), _points.end(), compare1);
			int nextDim = (divideDim + 1) % 2;
			data = *(_points[halfSize]);
			leftNode = new Node{};
			leftNode->divideDim = nextDim;
			leftNode->build(std::vector<Eigen::Vector2d*>(_points.begin(), _points.begin() + halfSize));
			if (_points.size() > 2) {
				rightNode = new Node{};
				rightNode->divideDim = nextDim;
				rightNode->build(std::vector<Eigen::Vector2d*>(_points.begin() + halfSize + 1, _points.end()));
			}
		}

		double dist(const Eigen::Vector2d& _p) {
			return (_p - data).norm();
		}

		bool interest(const Eigen::Vector2d& _p, double& _minD) {
			double d = std::abs(_p[divideDim] - data[divideDim]);
			if (d < _minD)
				return true;
			return false;
		}

		void search2(const Eigen::Vector2d& _p,	std::stack<Node*>& _rets) {
			_rets.push(this);
			if (_p[divideDim] <= data[divideDim] && leftNode)
				leftNode->search2(_p, _rets);
			else if (_p[divideDim] >= data[divideDim] && rightNode)
				rightNode->search2(_p, _rets);
		}

		void search(const Eigen::Vector2d& _p, const int& _size, const double& _squareR,
			std::vector<std::pair<double, Eigen::Vector2d>>& _rets) {
			double d = (_p - data).squaredNorm();
			if (d <= _squareR) {
				_rets.emplace_back(d, data);
				std::sort(_rets.begin(), _rets.end(), compareSearch);
				if (_rets.size() > _size)
					_rets.erase(_rets.end());
			}
			if (_p[divideDim] <= data[divideDim] && leftNode)
				leftNode->search(_p, _size, _squareR, _rets);
			if (_p[divideDim] >= data[divideDim] && rightNode)
				rightNode->search(_p, _size, _squareR, _rets);
		}

		~Node() {
			//std::cout << "delete: \n" << data << std::endl;
			if(leftNode)
				delete leftNode;
			if(rightNode)
				delete rightNode;
		}
	};

	KdTree2d();
	~KdTree2d();

	bool build(std::vector<Eigen::Vector2d>& _points);
	int knnSearch(const Eigen::Vector2d& _p, const int _size,
		std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists);
	int radiusSearch(const Eigen::Vector2d& _p, const double _r,
		std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists);
	int search(const Eigen::Vector2d& _p, const int _size, const double _r,
		std::vector<Eigen::Vector2d>& _points, std::vector<double>& _dists);

private:


private:

	Node* root = nullptr;
};




NSP_SLAM_LYJ_MATH_END

#endif //SLAM_LYJ_KDTREE_H
