CMAKE_MINIMUM_REQUIRED( VERSION 2.8.3 )
PROJECT( demo_client ) #设定工程名

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
		roscpp sensor_msgs std_msgs
		cv_bridge image_transport detect)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
		${CUDA_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS})

catkin_package(
		CATKIN_DEPENDS image_transport
		roscpp sensor_msgs cv_bridge
		detect)

AUX_SOURCE_DIRECTORY(./src OBJ_SRC)

message(${OBJ_SRC})
ADD_EXECUTABLE( m_client ${OBJ_SRC} )
target_link_libraries(m_client m  pthread ${OpenCV_LIBS} ${catkin_LIBRARIES})
