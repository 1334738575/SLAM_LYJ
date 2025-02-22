
cmake_minimum_required(VERSION 3.20)
project(examplePy)
#pybind11
set(pybind11_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/pybind11/install/share/cmake/pybind11")
find_package(pybind11 REQUIRED)
if(${pybind11_FOUND})
    find_package(Python REQUIRED COMPONENTS Interpreter Development)
    if(${Python_FOUND})
        message(STATUS "find python: ${Python_EXECUTABLE}")
        message(STATUS "find python: ${Python_INCLUDE_DIRS}")
        message(STATUS "find python: ${Python_LIBRARIES}")
        message(STATUS "find python: ${Python_VERSION}")
    else()
        message(FATAL_ERROR "find python fail")
    endif()
    add_library(${PROJECT_NAME} SHARED examplePy.cpp)
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".pyd")
    target_include_directories(${PROJECT_NAME}
        PRIVATE ${pybind11_INCLUDE_DIR}
        PRIVATE ${Python_INCLUDE_DIRS}
    )
    # pybind11_add_module()
    target_link_libraries(${PROJECT_NAME}
        PRIVATE ${Python_LIBRARIES}
        PUBLIC SLAM_LYJ
    )
endif()
