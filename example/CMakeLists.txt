add_executable(${PROJECT_NAME} example1.cpp)
# file(COPY ${CMAKE_BINARY_DIR}/src/Release/SLAM_LYJ.dll DESTINATION ${CMAKE_BINARY_DIR}/example/Release)
target_link_libraries(${PROJECT_NAME} SLAM_LYJ QT_LYJ)
# target_link_libraries(${PROJECT_NAME} SLAM_LYJ ${OpenCV_LIBS} Ceres::ceres ${VULKAN_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} glm::glm)
# set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER example)

# set exe output path
if(DEFINED LIBRARY_LYJ_OUTPUT_PATH)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_LYJ_OUTPUT_PATH}
    )
endif()