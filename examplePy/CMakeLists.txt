
cmake_minimum_required(VERSION 3.20)
project(examplePy)
#pybind11
set(pybind11_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/pybind11/install/share/cmake/pybind11")
find_package(pybind11 REQUIRED)
if(${pybind11_FOUND})
    add_library(${PROJECT_NAME} SHARED examplePy.cpp)
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".pyd")
    set(PYTHONPATH "F:/conda3/conda")
    target_include_directories(${PROJECT_NAME}
        PRIVATE ${pybind11_INCLUDE_DIR}
        PRIVATE ${PYTHONPATH}/include
    )
    # pybind11_add_module()
    target_link_libraries(${PROJECT_NAME}
        PRIVATE ${PYTHONPATH}/libs/python311.lib
        PUBLIC SLAM_LYJ
    )
endif()
