#ifndef SLAM_LYJ_PREDEFINE_H
#define SLAM_LYJ_PREDEFINE_H

//lyj
#include "Base.h"
#include "common/Point.h"
#include "common/Line.h"
#include "common/Plane.h"
#include "common/KdTree.h"
#include "common/RANSAC.h"
#include "common/ThreadPool.h"
#include "common/Tensor.h"
#include "common/CommonAlgorithm.h"
#include "common/Grid.h"
#include "Pose.h"
#include "CameraModule.h"
#include "config.h"

NSP_SLAM_LYJ_BEGIN

#define SYS_VERSION 1
#define SYS_DEBUG

class GlobalInnerOption
{
public:
	std::string sysName = "";
	std::string sysHomePath = SLAM_LYJ_HOME_PATH;

public:
	GlobalInnerOption() {}
	~GlobalInnerOption() {}

	static GlobalInnerOption* get() { return &opt_; }

private:
	static GlobalInnerOption opt_;
};

#define LYJOPT GlobalInnerOption::get()

NSP_SLAM_LYJ_END



#endif