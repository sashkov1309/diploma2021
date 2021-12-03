cmake_minimum_required(VERSION 3.19)
project(diploma_2021)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined -fsanitize-recover=all")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xarm_description REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(xarm_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(serial REQUIRED)
find_package(xarm_api REQUIRED)
find_package(Protobuf 3.6.1 EXACT REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${BULLET_INCLUDE_DIRS})

find_library(LIBSTATGRAB NAMES statgrab PATHS /usr/include)

add_library(${PROJECT_NAME}
        src/logic/perception/map_perceptor_node.cc
        src/logic/perception/perceptor/perceptor.cc
        src/logic/perception/perceptor/door_perceptor.cc
        )


ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_action
        std_msgs
        pugixml
        Boost
        ament_index_cpp
        geometry_msgs
        xarm_msgs
        Eigen3
        tf2_ros
        moveit_msgs
        moveit_core
        moveit_ros_planning_interface
        serial
        xarm_description
        pugixml
        trac_ik_lib
        OpenCV
        cv_bridge
        xarm_msgs
        xarm_api
        yaml-cpp
        Bullet
        )

target_link_libraries(${PROJECT_NAME} ${LIBSTATGRAB} ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${Protobuf_LIBRARIES}
        ${ZeroMQ_LIBRARY} ${ZeroMQPP_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${YAML_CPP_INCLUDE_DIRS})

###########################################################################
# Nodes
add_executable(main_node src/node/robot/main_node.cc)
target_link_libraries(main_node ${PROJECT_NAME})
###########################################################################

install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS
        main_node
        DESTINATION lib/${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
ament_package()
