CMAKE_MINIMUM_REQUIRED( VERSION 2.8.3 )
PROJECT( obj_detect_server ) #设定工程名

ADD_DEFINITIONS(-DLINUX_USE)
#ADD_DEFINITIONS(-DOPENCV -DGPU -DCUDNN -DLINUX_USE)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
		roscpp sensor_msgs std_msgs
		cv_bridge image_transport detect)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

#设定头文件目录
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
		${CUDA_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

catkin_package(
		CATKIN_DEPENDS image_transport
		roscpp sensor_msgs cv_bridge
		detect)

AUX_SOURCE_DIRECTORY(./src OBJ_SRC)

# 增加一个可执行的二进制
message(${OBJ_SRC})
ADD_EXECUTABLE( obj_detect ${OBJ_SRC} )
target_link_libraries(obj_detect m  pthread ${OpenCV_LIBS} ${catkin_LIBRARIES} darknet)
