#ifndef SLAM_LYJ_GRID_H
#define SLAM_LYJ_GRID_H

#include "base/Base.h"
#include <opencv2/opencv.hpp>


NSP_SLAM_LYJ_MATH_BEGIN


//w,h...
template<typename T, int DIM>
class Grid
{
public:
	typedef Eigen::Matrix<T, DIM, 1> VecTemX;
	Grid() {
		clear();
	}
	Grid(const int _resolution, const std::vector<int>& _sizes, const VecTemX& _center,
		const std::vector<VecTemX>& _points) {
		clear();
		if (_resolution <= 0 || (int)_sizes.size() != DIM)
			return;
		resolution_ = _resolution;
		center_ = _center;
		int totalSize = 1;
		for (int i = 0; i < DIM; ++i) {
			gridSize_[i] = _sizes[i] / resolution_ + 1;
			totalSize *= gridSize_[i];
		}
		inds_.resize(totalSize);
		std::vector<int> inds(DIM, 0);
		int ind = 0;
		int t = 1;
		for (int i = 0; i < (int)_points.size(); ++i) {
			for (int j = 0; j < DIM; ++j) {
				inds[j] = static_cast<int>(_points[i][j] - center_[j]) / resolution_ + gridSize_[j]/2;
			}
			ind = 0;
			t = 1;
			for (int ii = 0; ii < DIM; ++ii) {
				ind += t * inds[ii];
				t *= gridSize_[ii];
			}
			inds_[ind].push_back(i);
		}
	}
	Grid(const int& _resolution, const std::vector<VecTemX>& _points) {
		clear();
		if (_resolution <= 0)
			return;
		VecTemX maxVs;
		VecTemX minVs;
		minVs = maxVs = _points[0];
		for (int i = 1; i < (int)_points.size(); ++i) {
			for (int j = 0; j < DIM; ++j) {
				if(maxVs[j] < _points[i][j])
					maxVs[j] = _points[i][j];
				else if(minVs[j] > _points[i][j])
					minVs[j] = _points[i][j];
			}
		}
		center_ = (maxVs + minVs) / 2;
		resolution_ = _resolution;
		int totalSize = 1;
		for (int i = 0; i < DIM; ++i) {
			gridSize_[i] = static_cast<int>(maxVs[i] - minVs[i]) / resolution_ + 1;
			totalSize *= gridSize_[i];
		}
		inds_.resize(totalSize);
		std::vector<int> inds(DIM, 0);
		int ind = 0;
		int t = 1;
		for (int i = 0; i < (int)_points.size(); ++i) {
			for (int j = 0; j < DIM; ++j) {
				inds[j] = static_cast<int>(_points[i][j] - minVs[j]) / resolution_;
			}
			ind = 0;
			t = 1;
			for (int ii = 0; ii < DIM; ++ii) {
				ind += t * inds[ii];
				t *= gridSize_[ii];
			}
			inds_[ind].push_back(i);
		}
	}
	~Grid() {}

	void clear() {
		resolution_ = 0;
		memset(gridSize_, 0, sizeof(int) * DIM);
		inds_.clear();
		center_.setZero();
	}

	virtual const std::list<int>& getIndsInOne(const std::vector<int>& _inds) {
		int ind = 0;
		int t = 1;
		for (int ii = 0; ii < DIM; ++ii) {
			ind += t * _inds[ii];
			t *= gridSize_[ii];
		}
		return inds_[ind];
	}
	virtual const std::list<int>& getIndsNear(const VecTemX& _point) {
		std::vector<int> inds(DIM, 0);
		for (int j = 0; j < DIM; ++j) {
			inds[j] = static_cast<int>(_point[j] - center_[j]) / resolution_ + gridSize_[j]/2;
			if (inds[j] > gridSize_[j])
				return emptyInds_;
		}
		return getIndsInOne(inds);
	}
	virtual int getResolution() const { return resolution_; }

protected:
	int resolution_ = 0;
	int gridSize_[DIM];
	std::vector<std::list<int>> inds_;
	std::list<int> emptyInds_;
	VecTemX center_;
};

class Grid2Df : public Grid<float, 2>
{
public:
	Grid2Df() {}
	Grid2Df(const int& _resolution, const int _rows, const int _cols,
		const std::vector<cv::KeyPoint>& _points) {
		clear();
		if (_resolution <= 0)
			return;
		resolution_ = _resolution;
		center_ = Eigen::Vector2f(_rows / 2, _cols / 2);
		int totalSize = 1;
		gridSize_[0] = _cols / resolution_ + 1;
		totalSize *= gridSize_[0];
		gridSize_[1] = _rows / resolution_ + 1;
		totalSize *= gridSize_[1];
		inds_.resize(totalSize);
		std::vector<int> inds(2, 0);
		int ind = 0;
		int t = 0;
		for (int i = 0; i < (int)_points.size(); ++i) {
			inds[0] = static_cast<int>(_points[i].pt.x - center_[0]) / resolution_ + gridSize_[0] / 2;
			inds[1] = static_cast<int>(_points[i].pt.y - center_[1]) / resolution_ + gridSize_[1] / 2;
			ind = 0;
			t = 1;
			for (int ii = 0; ii < 2; ++ii) {
				ind += t * inds[ii];
				t *= gridSize_[ii];
			}
			inds_[ind].push_back(i);
		}
	}
	~Grid2Df() {}


private:

};





NSP_SLAM_LYJ_MATH_END


#endif //SLAM_LYJ_GRID_H