# project(cmdline)

# add_library(${PROJECT_NAME} INTERFACE)
#
# target_include_directories(${PROJECT_NAME} INTERFACE .)
#
# add_executable(test test.cpp)
# target_link_directories(test ${PROJECT_NAME})
#
# add_executable(test2 test2.cpp)
# target_link_directories(test2 ${PROJECT_NAME})
#
# include_directories(${PROJECT_SOURCE_DIR})

# set(SOURCE_FILES
#     ${PROJECT_SOURCE_DIR}/test.cpp
#     ${PROJECT_SOURCE_DIR}/test2.cpp
# )

add_executable(test1 test.cpp)
add_executable(test2 test2.cpp)
