cmake_minimum_required(VERSION 3.8)
project(bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/ros/humble/include")

# find dependencies
find_package(ament_cmake)
#set(ament_cmake_FOUND FALSE)
if (${ament_cmake_FOUND})
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  message("# Found ROS2")
#  find_package(teensy_bridge_messages REQUIRED)
#  set(EXTRA_CC_FLAGS "-DUSE_ROS2")
  add_compile_options(-DUSE_ROS2)
else()
  message("# Failed to find ROS2 - compiling without ROS2 support")
endif()
find_package(OpenCV REQUIRED )
find_package(Threads REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 only ??
if (${CPU} MATCHES "armv7l")
   message("# Is a RASPBERRY ${CPU}")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI")
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("# Not a RASPBERRY ${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++17 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")


add_executable(hw_bridge src/main.cpp
      src/utime.cpp
      src/ujoy.cpp
      src/userverport.cpp
      src/userverclient.cpp
      src/uconsole.cpp
      src/urun.cpp
      src/uteensy.cpp
      src/udata.cpp
      src/uhandler.cpp
      src/udataitem.cpp
      src/ulogfile.cpp
      src/tcpCase.cpp
      src/ubridge.cpp
      src/usource.cpp
      src/usubscribe.cpp
      src/uini.cpp
      src/uros2.cpp
      src/uhost.cpp)
if(ament_cmake_FOUND)
#  ament_target_dependencies(tbridge rclcpp std_msgs teensy_bridge_messages geometry_msgs tf2)
  ament_target_dependencies(hw_bridge rclcpp std_msgs geometry_msgs tf2)
endif()


target_link_libraries(hw_bridge ${CMAKE_THREAD_LIBS_INIT} readline)

install(TARGETS
  hw_bridge
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(ament_cmake_FOUND)
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
  ament_package()
endif()

