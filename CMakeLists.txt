cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp
                                        sensor_msgs cmake_modules tf2_ros)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

catkin_package(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} DEPENDS EIGEN3)
option(ASSERT_CHECKS "Enable assert" ON)

file(GLOB_RECURSE ALL_HEADERS "src/*.h")
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp")
list(REMOVE_ITEM ALL_SOURCES ${ALL_EXECUTABLES})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(sf_lib STATIC ${ALL_HEADERS} ${ALL_SOURCES})
add_executable(sensor_fusion src/sensor_fusion_main.cpp)

if (ASSERT_CHECKS)
  target_compile_definitions(sensor_fusion PUBLIC ENABLE_ASSERT=1)
endif()

# Specify libraries to link a library or executable target against
target_link_libraries(sf_lib ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY})
target_link_libraries(sensor_fusion sf_lib)
