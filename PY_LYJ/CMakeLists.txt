
cmake_minimum_required(VERSION 3.20)
project(PY_LYJ)
#pybind11
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

    macro(GenLibPy LIBNAME)
        set(PyName ${LIBNAME}_Py)
        add_library(${PyName} SHARED ${PyName}.cpp)
        set_target_properties(${PyName} PROPERTIES SUFFIX ".pyd")
        target_include_directories(${PyName}
            PRIVATE ${pybind11_INCLUDE_DIR}
            PRIVATE ${Python_INCLUDE_DIRS}
        )
        # pybind11_add_module()
        if(TARGET ${LIBNAME})
            target_link_libraries(${PyName}
                PRIVATE ${Python_LIBRARIES}
                PUBLIC ${LIBNAME}
            )
        else()
            target_link_libraries(${PyName}
                PRIVATE ${Python_LIBRARIES}
            )
        endif()

        # set dll, lib and exe output path
        if(DEFINED LIBRARY_LYJ_OUTPUT_PATH)
            set_target_properties(${PyName} PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
            ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
            RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
            )
        endif()
    endmacro(GenLibPy)

    GenLibPy(example)
    GenLibPy(SLAM_LYJ)
    GenLibPy(QT_LYJ)
    
    # add_library(${PROJECT_NAME} SHARED examplePy.cpp)
    # set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".pyd")
    # target_include_directories(${PROJECT_NAME}
    #     PRIVATE ${pybind11_INCLUDE_DIR}
    #     PRIVATE ${Python_INCLUDE_DIRS}
    # )
    # # pybind11_add_module()
    # target_link_libraries(${PROJECT_NAME}
    #     PRIVATE ${Python_LIBRARIES}
    #     PUBLIC SLAM_LYJ
    #     PUBLIC QT_LYJ
    # )
    # # set dll, lib and exe output path
    # if(DEFINED LIBRARY_LYJ_OUTPUT_PATH)
    #     set_target_properties(${PROJECT_NAME} PROPERTIES
    #     LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    #     ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    #     RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    #     )
    # endif()


    # # generate pyi using command
    # add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}
    # COMMENT "copy pyd"
    # # COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generatePyi.py #path error, not use
    # # COMMENT "generate pyi"
    # )
endif()
