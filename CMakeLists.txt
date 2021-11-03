cmake_minimum_required(VERSION 3.5)
project(zed_multi_camera)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

IF(NOT MSVC)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zed_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
    
add_executable(${PROJECT_NAME} src/multi_camera.cpp)
add_definitions(-std=c++14 -O3)
ament_target_dependencies(${PROJECT_NAME} ZED OpenCV CUDA ament_cmake rclcpp sensor_msgs zed_components nav_msgs)
target_link_libraries(${PROJECT_NAME} zed_odom)

add_library(zed_odom src/zed_odom.cpp)
ament_target_dependencies(zed_odom ZED OpenCV CUDA ament_cmake rclcpp sensor_msgs zed_components nav_msgs tf2_ros)
 
if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${SPECIAL_OS_LIBS})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY} ${SPECIAL_OS_LIBS})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZED_LIBS} ${SPECIAL_OS_LIBS} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} zed_odom
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME})

ament_package()
