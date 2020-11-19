cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs diagnostic_updater tf netft_rdt_msgs)
find_package(Boost)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs diagnostic_msgs diagnostic_updater tf netft_rdt_msgs
    #  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_driver.h
    ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_bias.h
    ${INCLUDE_DIR}/netft_rdt_driver/ft_listener.h
    )

set(SRC_FILES ${SRC_DIR}/netft_rdt_driver.cpp
    ${SRC_DIR}/netft_rdt_bias.cpp
    ${SRC_DIR}/ft_listener.cpp
    )


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${INCLUDE_DIR}
    ${KALMAN_INCLUDE}
)


add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(netft_node ${SRC_DIR}/netft_node.cpp)
target_link_libraries(netft_node ${PROJECT_NAME} boost_program_options)

