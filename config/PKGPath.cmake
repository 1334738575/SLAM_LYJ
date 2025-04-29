list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Vulkan")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Glfw")

# set(ZLIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/zlib")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dependencies/zlib")

# set(LZ4_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/lz4")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dependencies/lz4")
# set(LZ4_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/lz4/lib/cmake/lz4")

# set(PNG_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/libpng")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dependencies/libpng")
# set(PNG_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/libpng/lib/cmake/PNG")

list(APPEND CMAKE_MODULE_PATH "D:/SLAM_LYJ/dependencies/Glew")

set(Boost_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Boost/lib/cmake/Boost-1.86.0")
set(glm_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Glm099/glm/cmake/glm")
set(Eigen3_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Eigen/share/eigen3/cmake")
set(Ceres_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Ceres/lib/cmake/Ceres")
set(OpenCV_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Opencv/lib")
set(cJSON_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/cJSON/lib/cmake/cJSON")
set(flann_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/flann/lib/cmake/flann")
set(VTK_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/vtk/lib/cmake/VTK-9.4")
set(PCL_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/pcl/cmake")
set(pybind11_DIR "${CMAKE_CURRENT_LIST_DIR}/../dependencies/pybind11/install/share/cmake/pybind11")


set(SHADERCOMPILER "${CMAKE_CURRENT_LIST_DIR}/../dependencies/Vulkan121981/SDK121981/Bin/glslangValidator.exe")
