cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)


find_package(catkin REQUIRED COMPONENTS genmsg roscpp diagnostic_updater std_msgs geometry_msgs message_generation tf)
find_package(Boost)


add_service_files(FILES
    String_cmd.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs diagnostic_updater diagnostic_msgs message_runtime tf
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_driver.h
                 ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_bias.h
                 ${INCLUDE_DIR}/netft_rdt_driver/ft_listener.h
)

set(SRC_FILES   ${SRC_DIR}/netft_rdt_driver.cpp
                ${SRC_DIR}/netft_rdt_bias.cpp
                ${SRC_DIR}/ft_listener.cpp

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${KALMAN_INCLUDE}
)


add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(netft_node ${SRC_DIR}/netft_node.cpp      )
target_link_libraries(netft_node ${PROJECT_NAME} boost_program_options)

