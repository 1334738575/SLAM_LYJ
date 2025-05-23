cmake_minimum_required(VERSION 3.10)

###############################setting####################################
#cmake setting
project(SLAM_LYJ_exe)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
if(WIN32)
    if(MSVC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
        # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od /Ob0")
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
        add_link_options(/DEBUG) #生成调试信息（测试在visual studio中有效）
    endif()
elseif(LINUX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g")
endif()
# set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(USE_BOOST "Enable boost." OFF)
option(USE_OLD_SHADER "Use old shader or complie new shader." ON)
set(LIBRARY_LYJ_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/example)

#find package and add functions
include(config/PKGPath.cmake)


###############################find_package####################################
#vulkan
find_package(Vulkan)
include_directories(${VULKAN_INCLUDE_DIRS})

#glfw
find_package(Glfw)
include_directories(${GLFW_INCLUDE_DIRS})

#glm
find_package(glm)

#boost
if(USE_BOOST)
    find_package(Boost REQUIRED COMPONENTS serialization)
    if(${Boost_FOUND})
        add_definitions(-DUSEBOOST)
        include_directories(${Boost_INCLUDE_DIRS})
    endif(${Boost_FOUND})
endif()


#eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#ceres
find_package(Ceres REQUIRED)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#openmp
find_package(OpenMP REQUIRED) 
# if(${OpenMP_CXX_FOUND})
#     add_compile_options(${OpenMP_CXX_FLAGS})
#     message("find OpenMP: ${OpenMP_CXX_LIBRARIES}")
#     message("find OpenMP: ${OpenMP_CXX_FLAGS}")
# endif()

#pcl
find_package(PCL)
# message(FATAL_ERROR ${PCL_INCLUDE_DIRS})
# message("D:/SLAM_LYJ/dependencies/pcl/include/pcl-1.15")
# message("D:/SLAM_LYJ/dependencies/flann/include")
include_directories(${PCL_INCLUDE_DIRS}) #可能无法在visual studio中显示，需要手动添加（不影响编译）

#QT
# message("E:/anaconda3/Library/include/qt/QtCore")
# message("E:/anaconda3/Library/include/qt/QtGui")
# message("E:/anaconda3/Library/include/qt/QtWidgets")
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

#GLEW
find_package(Glew)
include_directories(${GLEW_INCLUDE_DIRS})

#cuda
enable_language(CUDA)
find_package(CUDA)
# message(FATAL_ERROR "cuda version:"${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CUDA_ARCHITECTURES 89)

###############################sub_projects####################################
#add source
set(MULTI_DLL 0)
add_definitions(-DLYJDEBUG)
include_directories(src)
add_subdirectory(src)
file(GLOB SLAM_LYJ_CONFIG
    config/*.h)
source_group(config/${_source_path_msvc} FILES ${SLAM_LYJ_CONFIG})

include_directories(viewer)
add_subdirectory(viewer)

#add pytest
add_subdirectory(PY_LYJ)

#add executable
add_subdirectory(example)


###############################importlibrary####################################
# set(DLL_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.dll)
# # file(EXISTS ${DLL_FILE} DLL_EXISTS)
# # if(DLL_EXSITS)
# if(EXISTS ${DLL_FILE})
#     file(COPY ${DLL_FILE} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/example/Release)
# endif()
# add_custom_command(OUTPUT COPY_RES
# # COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/etc
# COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/log.txt ${CMAKE_CURRENT_SOURCE_DIR}/etc
# )
add_custom_target(CopyDll
# COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/Release ${CMAKE_CURRENT_BINARY_DIR}/example/Release
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.dll ${CMAKE_CURRENT_BINARY_DIR}/example/Release/SLAM_LYJ.dll
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.pdb ${CMAKE_CURRENT_BINARY_DIR}/example/Release/SLAM_LYJ.pdb
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.exp ${CMAKE_CURRENT_BINARY_DIR}/example/Release/SLAM_LYJ.exp
)