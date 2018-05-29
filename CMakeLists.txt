cmake_minimum_required(VERSION 2.8.3)
project(dream_babbling)

if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(ColourBold "${Esc}[1m")
    set(Red "${Esc}[31m")
    set(Green "${Esc}[32m")
    set(Yellow "${Esc}[33m")
    set(Blue "${Esc}[34m")
    set(Magenta "${Esc}[35m")
    set(Cyan "${Esc}[36m")
    set(White "${Esc}[37m")
    set(BoldRed "${Esc}[1;31m")
    set(BoldGreen "${Esc}[1;32m")
    set(BoldYellow "${Esc}[1;33m")
    set(BoldBlue "${Esc}[1;34m")
    set(BoldMagenta "${Esc}[1;35m")
    set(BoldCyan "${Esc}[1;36m")
    set(BoldWhite "${Esc}[1;37m")
endif ()

#Usage example: catkin_make -DROBOT=baxter -DCATKIN_WHITELIST_PACKAGES=dream_babbling
#By default the target is the crustcrawler
set(ROBOT "crustcrawler" CACHE STRING "The targeted robot for the experiment.")
message(" ")
message("${BoldMagenta}-- Building experiment for the ${ROBOT} --${ColourReset}")
message(" ")

file(GLOB_RECURSE include_data_files include/*)
file(GLOB_RECURSE src_files src/*)

set(CMAKE_INCLUDE_DIRECTORIES_BEFORE ON)
set(CMAKE_MODULE_PATH ~/dream/arm_experiment_ws/cmake_modules)

set(CLASS_EVAL_SOURCE_FILES src/classifier_eval.cpp)
set(CATKIN_COMMON_COMPONENTS cafer_core
                             cv_bridge
                             roscpp
                             rospy
                             std_msgs
                             message_generation
                             image_transport
                             image_geometry
                             tf
                             actionlib
                             actionlib_msgs
                             rgbd_utils
                             message_filters
                             geometry_msgs
                             sensor_msgs
                             relevance_map
                            )



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

if (${ROBOT} STREQUAL "fake")
    find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMMON_COMPONENTS})
elseif (${ROBOT} STREQUAL "crustcrawler")
    find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMMON_COMPONENTS} moveit_ros_planning_interface crustcrawler_mover_utils)
elseif (${ROBOT} STREQUAL "baxter")
    find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMMON_COMPONENTS} moveit_ros_planning_interface baxter_mover_utils)
elseif (${ROBOT} STREQUAL "pr2")
    find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMMON_COMPONENTS} moveit_ros_planning_interface pr2_mover_utils)
endif ()

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")
# set(CMAKE_CXX_COMPILER /usr/local/bin/clang++)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(yaml_cpp REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LOCAL_LIB_DIR /usr/local/ CACHE STRING "local libraries directory for iagmm and image_processing")
set(IAGMM_INCLUDE_DIR ${LOCAL_LIB_DIR}/include)
set(IAGMM_LIBRARIES ${LOCAL_LIB_DIR}/lib/libiagmm.so)
set(image_processing_INCLUDE_DIR ${LOCAL_LIB_DIR}/include/)
set(image_processing_LIBRARIES ${LOCAL_LIB_DIR}/lib/libimage_processing.so)

find_library(BASE64 NAMES b64)

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)


#find_path(IMAGE_PROCESSING_INCLUDE_DIR image_processing/MotionDetection.h)
#find_library(IMAGE_PROCESSING_LIBRARY NAMES image_processing)

#if (${ROBOT} STREQUAL "crustcrawler")
#    set(ROBOT_SOURCE_FILES src/controller_node.cpp)
#    set(ROBOT_LIBS ${catkin_LIBRARIES} real_controllers dynamixel)
if (${ROBOT} STREQUAL "crustcrawler")
    set(ROBOT_SOURCE_FILES src/crustcrawler/crustcrawler_moveit_utils.cpp)
    set(ROBOT_LIBS ${catkin_LIBRARIES})
elseif (${ROBOT} STREQUAL "baxter")
    set(ROBOT_SOURCE_FILES src/baxter/baxter_moveit_utils.cpp)
    set(ROBOT_LIBS ${catkin_LIBRARIES})
elseif (${ROBOT} STREQUAL "pr2")
    set(ROBOT_SOURCE_FILES src/pr2/pr2_moveit_utils.cpp)
    set(ROBOT_LIBS ${catkin_LIBRARIES})
elseif (${ROBOT} STREQUAL "fake")
    set(ROBOT_SOURCE_FILES src/controller_node_fake.cpp)
    set(ROBOT_LIBS ${catkin_LIBRARIES})
endif ()


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        rgbd_motion_data.msg
        joints_values.msg
        motion_rect.msg
        dataset.msg
        sv_feature.msg
        gmm_archive.msg
        endeffector_pose.msg
        sync_dataset.msg
        target_info.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        is_moving.srv
)

## Generate actions in the 'action' folder
add_action_files(
        FILES
        pose_goal.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        actionlib_msgs
        geometry_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if(${ROBOT} STREQUAL "crustcrawler")
    set(ROBOT_DEPEND crustcrawler_mover_utils)
elseif(${ROBOT} STREQUAL "baxter")
    set(ROBOT_DEPEND baxter_mover_utils)
elseif(${ROBOT} STREQUAL "pr2")
    set(ROBOT_DEPEND pr2_mover_utils)
endif()
catkin_package(
        INCLUDE_DIRS
        #  LIBRARIES dream_babbling
        CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs message_runtime cafer_core rgbd_utils gazebo_msgs sensor_msgs relevance_map  ${ROBOT_DEPEND}
        DEPENDS ${IMAGE_PROCESSING_LIBRARY}
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpencCV_INCLUDE_DIRS}
#        ${RGBD_UTILS_INCLUDE_DIR}
BEFORE SYSTEM        ${image_processing_INCLUDE} ${IAGMM_INCLUDE_DIR}
)

message(STATUS "catkin include" ${catkin_INCLUDE_DIRS})

# add_library(IMAGE_PROCESSING STATIC IMPORTED)
# point the imported target at the real file
# set_property(TARGET IMAGE_PROCESSING PROPERTY IMPORTED_LOCATION /usr/local/lib/libimage_processing.a)

## Declare a C++ library
# add_library(dream_babbling
#   src/${PROJECT_NAME}/dream_babbling.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dream_babbling ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(motion_detector_node src/motion_detector_node.cpp)
add_executable(babbling_node src/babbling_node.cpp include/globals.h)
#add_executable(fake_babbling_node src/fake_babbling_node.cpp include/globals.h)
add_executable(controller_node ${ROBOT_SOURCE_FILES})
add_executable(db_manager_node src/db_manager_node.cpp ${include_data_files})
add_executable(supervisor_node src/supervisor_node.cpp)
add_executable(class_eval_node src/classifier_eval.cpp)
add_executable(data_sync_node src/data_sync_node.cpp)

#TOOLS
add_executable(display_workspace tools/display_workspace.cpp)
add_executable(saliency_map_viz tools/saliency_map_viz.cpp)
add_executable(center_color_vccs tools/center_color_vccs.cpp)
#add_executable(replay_babbling tools/replay_babbling.cpp)
add_executable(motion_detection_test tools/motion_detection_test.cpp)
add_executable(choice_heat_map tools/choice_heat_map.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(babbling_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(motion_detector_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(babbling_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(fake_babbling_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(db_manager_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(class_eval_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(center_color_vccs dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(replay_babbling dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(saliency_map_viz dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(display_workspace dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(data_sync_node dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(motion_detection_test dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(choice_heat_map dream_babbling_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(motion_detector_node ${catkin_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} tbb)
target_link_libraries(motion_detection_test ${catkin_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} tbb)
target_link_libraries(babbling_node ${catkin_LIBRARIES}  ${IAGMM_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES} yaml-cpp.so tbb)
#target_link_libraries(fake_babbling_node ${catkin_LIBRARIES}  ${IAGMM_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES} yaml-cpp.so tbb)
target_link_libraries(db_manager_node ${catkin_LIBRARIES} ${BASE64} yaml-cpp.so)
target_link_libraries(controller_node ${ROBOT_LIBS})
target_link_libraries(supervisor_node ${catkin_LIBRARIES})
target_link_libraries(display_workspace ${catkin_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} tbb)
target_link_libraries(saliency_map_viz ${catkin_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp.so ${IAGMM_LIBRARIES} tbb ${BOOST_LIBRARIES})
target_link_libraries(center_color_vccs ${catkin_LIBRARIES} ${image_processing_LIBRARIES} ${PCL_LIBRARIES} tbb)
target_link_libraries(data_sync_node ${catkin_LIBRARIES} yaml-cpp.so ${OpenCV_LIBRARIES})
target_link_libraries(class_eval_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${IAGMM_LIBRARIES} ${image_processing_LIBRARIES} yaml-cpp.so tbb ${BOOST_LIBRARIES})
#target_link_libraries(replay_babbling ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${IAGMM_LIBRARIES} ${image_processing_LIBRARIES} yaml-cpp.so tbb ${BOOST_LIBRARIES})
target_link_libraries(choice_heat_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${IAGMM_LIBRARIES} ${image_processing_LIBRARIES} yaml-cpp.so tbb)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dream_babbling dream_babbling_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dream_babbling.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
