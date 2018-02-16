cmake_minimum_required (VERSION 2.8)
project (xamlasysmon)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

#FIND_PACKAGE(OpenCV REQUIRED)
find_package(Torch REQUIRED)
find_package(catkin REQUIRED COMPONENTS xamla_sysmon_msgs)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(src
  "${SOURCE_DIR}/statuscodes.cpp"
)

include_directories(
  ${Torch_INSTALL_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${xamla_sysmon_msgs_INCLUDES}
)

link_directories(
  ${Torch_INSTALL_LIB}
)

add_library(xamlasysmon SHARED ${src})

target_link_libraries(xamlasysmon TH ${catkin_LIBRARIES} ${xamla_sysmon_msgs_LIBS})

install(TARGETS xamlasysmon LIBRARY DESTINATION ${Torch_INSTALL_LUA_CPATH_SUBDIR})
install(DIRECTORY "lua/" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/xamla_sysmon" FILES_MATCHING PATTERN "*.lua")
