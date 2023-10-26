# getting-started
add_executable(${PROJECT_NAME} main.cpp funcLib.cpp)
# target_link_libraries(${PROJECT_NAME} ${RSSDK2_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY} ${PCL_LIBRARIES})
