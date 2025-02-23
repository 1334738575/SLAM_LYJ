
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
    # set dll and lib output path
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../build/example
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../build/example
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../build/example
    )
    # # generate pyi using command
    # add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}
    # COMMENT "copy pyd"
    # # COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generatePyi.py #path error, not use
    # # COMMENT "generate pyi"
    # )
endif()
