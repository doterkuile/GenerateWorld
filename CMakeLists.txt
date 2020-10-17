cmake_minimum_required(VERSION 2.8.3)
project(generate_world)


find_package(catkin REQUIRED COMPONENTS
    roscpp cmake_modules)
find_package(TinyXML2 REQUIRED)
set(CMAKE_CXX_FLAGS "-Wno-error=comment -Wno-error=shadow")



catkin_package(
    DEPENDS
    TinyXML2
    )

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
                   )

add_executable(generate_world_node
     src/generate_world.cpp
     src/world.cpp
     src/Block.cpp)

add_executable(stepping_stones_node
     src/stepping_stones.cpp
     src/world.cpp
     src/Block.cpp)


add_executable(create_stairs_node
     src/create_stairs.cpp
     src/world.cpp
     src/Block.cpp)

target_link_libraries(generate_world_node
   ${catkin_LIBRARIES}
   ${TinyXML2_LIBRARIES}
 )


target_link_libraries(stepping_stones_node
   ${catkin_LIBRARIES}
   ${TinyXML2_LIBRARIES}
 )

target_link_libraries(create_stairs_node
   ${catkin_LIBRARIES}
   ${TinyXML2_LIBRARIES}
 )
