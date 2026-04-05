
cmake_minimum_required(VERSION 3.20)

project(RunPythonInVenv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(PYBIND11_FINDPYTHON ON)

# set(PYTHON_ENV_ROOT "E:/anaconda3" CACHE PATH "Path to the target Python virtual/conda environment")
# set(PYTHON_ENV_ROOT "F:/AnacondaFile/envs/testEnv" CACHE PATH "Path to the target Python virtual/conda environment")
set(PYTHON_ENV_ROOT "F:/AnacondaFile/envs/surfel_splatting" CACHE PATH "Path to the target Python virtual/conda environment" FORCE)

if(PYTHON_ENV_ROOT)
    file(TO_CMAKE_PATH "${PYTHON_ENV_ROOT}" PYTHON_ENV_ROOT)

    set(Python3_ROOT_DIR "${PYTHON_ENV_ROOT}" CACHE PATH "" FORCE)

    if(WIN32)
        if(EXISTS "${PYTHON_ENV_ROOT}/python.exe")
            set(Python3_EXECUTABLE "${PYTHON_ENV_ROOT}/python.exe" CACHE FILEPATH "" FORCE)
        elseif(EXISTS "${PYTHON_ENV_ROOT}/Scripts/python.exe")
            set(Python3_EXECUTABLE "${PYTHON_ENV_ROOT}/Scripts/python.exe" CACHE FILEPATH "" FORCE)
        endif()
    endif()

    if(EXISTS "${PYTHON_ENV_ROOT}/Lib/site-packages/pybind11/share/cmake/pybind11")
        set(pybind11_DIR
            "${PYTHON_ENV_ROOT}/Lib/site-packages/pybind11/share/cmake/pybind11"
            CACHE PATH "" FORCE
        )
    endif()
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(pybind11_DIR "D:/SLAM_LYJ/dependencies/pybind11/install/share/cmake/pybind11")
find_package(pybind11 CONFIG REQUIRED)

add_executable(run_python_in_venv
    py3DGS.cpp
)

target_link_libraries(run_python_in_venv
    PRIVATE
        pybind11::embed
)


if(MSVC)
    set_target_properties(run_python_in_venv PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=${PYTHON_ENV_ROOT}")
endif()


