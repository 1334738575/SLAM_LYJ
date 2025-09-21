cmake_minimum_required(VERSION 3.20)

###############################setting####################################
#cmake setting
project(SLAM_LYJ_Bin)
set(CMAKE_CXX_STANDARD 17)
if(WIN32)
    if(MSVC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")

        # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od /Ob0")

        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi")

        # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Ob0")

        # set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
        add_link_options(/DEBUG) # 生成调试信息（测试在visual studio中有效）
    endif()
elseif(LINUX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g")

    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")
endif()

option(USE_BOOST "Enable boost." OFF)
option(USE_OLD_SHADER "Use old shader or complie new shader." ON)
# set(LIBRARY_LYJ_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/example)

# #find package and add functions
include(config/PKGPath.cmake)


###############################find_package####################################

# option(USE_OLD_SHADER "use old shader" OFF)
set(BINDIRECTORIES)
set(Boost_DIR "D:/SLAM_LYJ/dependencies/Boost/lib/cmake/Boost-1.86.0")
find_package(Boost  REQUIRED COMPONENTS serialization)
set(OpenCV_DIR "D:/SLAM_LYJ/dependencies/Opencv/lib")
find_package(OpenCV)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found.")
endif()
list(APPEND BINDIRECTORIES "${OpenCV_INSTALL_PATH}/bin;")
set(Eigen3_DIR "D:/SLAM_LYJ/dependencies/Eigen/share/eigen3/cmake")
find_package(Eigen3)
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# find_package(DBow2)
set(Ceres_DIR "D:/SLAM_LYJ/dependencies/Ceres/lib/cmake/Ceres")
find_package(Ceres REQUIRED)
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/Ceres/bin;D:/SLAM_LYJ/dependencies/Glog/bin;D:/SLAM_LYJ/dependencies/Gflags/Lib;")
list(APPEND CMAKE_MODULE_PATH "E:/vulkan")
find_package(Vulkan)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
list(APPEND BINDIRECTORIES "E:/anaconda3/pkgs/qt-main-5.15.2-h19c9488_10/Library/bin;")
list(APPEND CMAKE_MODULE_PATH "D:/testLyj/GLFW/glfw-3.4.bin.WIN64")
find_package(Glfw)
list(APPEND BINDIRECTORIES "D:/testLyj/GLFW/glfw-3.4.bin.WIN64/lib-vc2022;")
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/Glew/bin/Release/x64;")
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/libpng/bin;")
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/zlib/bin;")
list(APPEND BINDIRECTORIES "E:/anaconda3/pkgs/icu-73.1-h6c2663c_0/Library/bin;")
list(APPEND BINDIRECTORIES "E:/anaconda3/pkgs/zstd-1.5.5-hd43e919_2/Library/bin;")
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/flann/bin\;")
list(APPEND BINDIRECTORIES "D:/SLAM_LYJ/dependencies/lz4/bin\;")
list(APPEND CMAKE_MODULE_PATH "D:/testLyj/GLM/glm-1.0.1")
find_package(glm)
# #openmp
# find_package(OpenMP REQUIRED) 
# # if(${OpenMP_CXX_FOUND})
# #     add_compile_options(${OpenMP_CXX_FLAGS})
# #     message("find OpenMP: ${OpenMP_CXX_LIBRARIES}")
# #     message("find OpenMP: ${OpenMP_CXX_FLAGS}")
# # endif()
list(APPEND BINDIRECTORIES "D:/packages/DBoW3/bin;")

# #pcl
# find_package(PCL)
# # message(FATAL_ERROR ${PCL_INCLUDE_DIRS})
# # message("D:/SLAM_LYJ/dependencies/pcl/include/pcl-1.15")
# # message("D:/SLAM_LYJ/dependencies/flann/include")
# include_directories(${PCL_INCLUDE_DIRS}) #可能无法在visual studio中显示，需要手动添加（不影响编译）

###############################sub_projects####################################

# set(CUDA_LYJ_DIR "D:/testCmake/pro2/install/lib/cmake")
# find_package(CUDA_LYJ)
# list(APPEND BINDIRECTORIES "${CUDA_LYJ_DIR}/../../bin;")
# set(OPTIMIZE_LYJ_DIR "D:/testCmake/pro3/install/lib/cmake")
# find_package(OPTIMIZE_LYJ)
# list(APPEND BINDIRECTORIES "${OPTIMIZE_LYJ_DIR}/../../bin;")
# set(COMMON_LYJ_DIR "D:/testCmake/pro4/install/lib/cmake")
# find_package(COMMON_LYJ)
# list(APPEND BINDIRECTORIES "${COMMON_LYJ_DIR}/../../bin;")
# set(PATH_PLAN_LYJ_DIR "D:/testCmake/pro5/install/lib/cmake")
# find_package(PATH_PLAN_LYJ)
# list(APPEND BINDIRECTORIES "${PATH_PLAN_LYJ_DIR}/../../bin;")
# set(Vulkan_LYJ_DIR "D:/testCmake/pro6/install/lib/cmake")
# find_package(Vulkan_LYJ)
# list(APPEND BINDIRECTORIES "${Vulkan_LYJ_DIR}/../../bin;")
# set(ImageProcess_LYJ_DIR "D:/testCmake/ImageProcess_LYJ/install/lib/cmake")
# find_package(ImageProcess_LYJ)
# list(APPEND BINDIRECTORIES "${ImageProcess_LYJ_DIR}/../../bin;")
add_subdirectory(SLAM_LYJ_CUDA)
add_subdirectory(SLAM_LYJ_optimize)
add_subdirectory(SLAM_LYJ_COMMON)
add_subdirectory(SLAM_LYJ_pathplan)
add_subdirectory(SLAM_LYJ_qt)
add_subdirectory(SLAM_LYJ_src)
add_subdirectory(SLAM_LYJ_vulkan)
add_subdirectory(ImageProcess_LYJ)

# #add pytest
# add_subdirectory(PY_LYJ)


###############################importlibrary####################################
set(${PROJECT_NAME}_LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/Output)
add_executable(${PROJECT_NAME} example/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
${DBow2_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${VULKAN_INCLUDE_DIRS}
${GLFW_INCLUDE_DIRS}
${GLM_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
${Boost_LIBRARIES}
${DBow2_LIBRARIY}
${OpenCV_LIBS} 
${VULKAN_LIBRARIES}
${GLFW_LIBRARIES}
Ceres::ceres

# CUDA_LYJ::CUDA_LYJ
# OPTIMIZE_LYJ::OPTIMIZE_LYJ
# COMMON_LYJ::COMMON_LYJ
# PATH_PLAN_LYJ::PATH_PLAN_LYJ
# Vulkan_LYJ::Vulkan_LYJ
# ImageProcess_LYJ::ImageProcess_LYJ
CUDA_LYJ
OPTIMIZE_LYJ
COMMON_LYJ
PATH_PLAN_LYJ
Vulkan_LYJ
ImageProcess_LYJ
QT_LYJ
SLAM_LYJ_src
)

if(DEFINED ${PROJECT_NAME}_LIBRARY_OUTPUT_PATH)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        ARCHIVE_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
        RUNTIME_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_PATH}
    )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=${BINDIRECTORIES}")
