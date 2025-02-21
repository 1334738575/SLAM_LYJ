#ifndef SLAM_LYJ_H
#define SLAM_LYJ_H

#include <base/Base.h>

NSP_SLAM_LYJ_BEGIN

SLAM_LYJ_API void testCeres();
SLAM_LYJ_API void testOpenCV();
SLAM_LYJ_API void testEigen();
SLAM_LYJ_API void testVulkan();
SLAM_LYJ_API void testKdTree();
SLAM_LYJ_API void testPoint();
SLAM_LYJ_API void testArchive();
SLAM_LYJ_API void testTensor(); //-
SLAM_LYJ_API void testCommonAlgorithm(); //-
SLAM_LYJ_API void testRANSAC();
SLAM_LYJ_API void testLine();//-
SLAM_LYJ_API void testPlane();//-
SLAM_LYJ_API void testPose();
SLAM_LYJ_API void testCamera();
SLAM_LYJ_API void testTriangler();//-
SLAM_LYJ_API void testOptimizer();//-
SLAM_LYJ_API void testBitFlagVec();
SLAM_LYJ_API void testFrame(); //- contain feature extract and match
SLAM_LYJ_API void testIncrementalAgvAndVar();
SLAM_LYJ_API void testGrid();
SLAM_LYJ_API void testSTLPlus();
SLAM_LYJ_API void testBuffer();
SLAM_LYJ_API void testGlobalOption();

SLAM_LYJ_API int getVersion();

SLAM_LYJ_API void testThreadPool();





NSP_SLAM_LYJ_END

#endif