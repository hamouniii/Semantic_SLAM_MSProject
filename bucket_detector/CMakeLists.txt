cmake_minimum_required(VERSION 2.8.3)
project(ShapeColor_ObjectDetection)


### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)

set(VISION_BASED_POSE_ESTIMATION_SOURCE_DIR src/sources) 
set(VISION_BASED_POSE_ESTIMATION_INCLUDE_DIR src/include)

# recursively add all source files
FILE(GLOB_RECURSE VISION_BASED_POSE_ESTIMATION_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.cpp)
#MESSAGE(STATUS "sources for "  ${APP_NAME} " are : " ${PROJ_SOURCE_FILES})

FILE(GLOB_RECURSE VISION_BASED_POSE_ESTIMATION_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE_MODULE_PATH is: " ${CMAKE_MODULE_PATH})



find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs opencv_apps cv_bridge tf_conversions
                                        image_transport message_generation)


find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    ObjectInfo.msg
    DetectedObjects.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    )

catkin_package(
	INCLUDE_DIRS ${VISION_BASED_POSE_ESTIMATION_INCLUDE_DIR}
	LIBRARIES ShapeColor_ObjectDetection
        CATKIN_DEPENDS roscpp rospy std_msgs opencv_apps cv_bridge tf_conversions image_transport message_runtime
	DEPENDS OpenCV
)

include_directories(${VISION_BASED_POSE_ESTIMATION_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${VISION_BASED_POSE_ESTIMATION_SOURCE_FILES} ${VISION_BASED_POSE_ESTIMATION_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

## Declare a C++ executable
add_executable(ShapeColor_ObjectDetection_Node src/test/ShapeColor_ObjectDetection_Node.cpp)
add_dependencies(ShapeColor_ObjectDetection_Node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ShapeColor_ObjectDetection_Node ${PROJECT_NAME})
target_link_libraries(ShapeColor_ObjectDetection_Node ${catkin_LIBRARIES} ${OpenCV_LIBRARY})

add_executable(ObjectDetection_Node src/test/ObjectDetection_Node.cpp)
add_dependencies(ObjectDetection_Node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObjectDetection_Node ${PROJECT_NAME})
target_link_libraries(ObjectDetection_Node ${catkin_LIBRARIES} ${OpenCV_LIBRARY})



