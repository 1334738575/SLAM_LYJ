cmake_minimum_required(VERSION 3.10)

###############################setting####################################
#cmake setting
project(SLAM_LYJ_exe)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
if(WIN32)
    if(MSVC)
        # message(FATAL_ERROR "1")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
        # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od /Ob0")
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
    endif()
elseif(LINUX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g")
endif()
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(USE_BOOST "Enable boost." OFF)
option(USE_OLD_SHADER "Use old shader or complie new shader." ON)

#find package and add functions
include(cmake)


###############################find_package####################################
#vulkan
# set(${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Vulkan;${CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Vulkan")
# message(FATAL_ERROR ${CMAKE_MODULE_PATH})
find_package(Vulkan)
include_directories(${VULKAN_INCLUDE_DIRS})
# message(FATAL_ERROR ${VULKAN_LIBRARIES})

#glfw
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Glfw")
find_package(Glfw)
include_directories(${GLFW_INCLUDE_DIRS})
# message(FATAL_ERROR ${GLFW_INCLUDE_DIRS})

#glm
set(glm_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Glm099/glm/cmake/glm")
# message(FATAL_ERROR ${glm_DIR})
find_package(glm)

#boost
if(USE_BOOST)
    find_package(Boost REQUIRED COMPONENTS serialization)
    if(${Boost_FOUND})
        add_definitions(-DUSEBOOST)
        # message(FATAL_ERROR ${Boost_INCLUDE_DIRS})
        include_directories(${Boost_INCLUDE_DIRS})
    endif(${Boost_FOUND})
endif()


#eigen
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake;${CMAKE_MODULE_PATH}")
set(Eigen3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Eigen/share/eigen3/cmake")
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# message(FATAL_ERROR ${EIGEN3_INCLUDE_DIRS})

#ceres
set(Ceres_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Ceres/lib/cmake/Ceres")
find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})
# message(FATAL_ERROR ${CERES_INCLUDE_DIRS})

#opencv
# set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Opencv")
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Opencv/lib")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# message(FATAL_ERROR ${OpenCV_LIBS})


###############################sub_projects####################################
#add source
set(MULTI_DLL 0)
add_definitions(-DLYJDEBUG)
set(SHADERCOMPILER F:/Vulkan/SDK121981/Bin/glslangValidator.exe)
include_directories(src)
add_subdirectory(src)

#add pytest
add_subdirectory(examplePy)

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
# COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/etc
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.dll ${CMAKE_CURRENT_BINARY_DIR}/example/Release
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.pdb ${CMAKE_CURRENT_BINARY_DIR}/example/Release
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/src/Release/SLAM_LYJ.exp ${CMAKE_CURRENT_BINARY_DIR}/example/Release
)