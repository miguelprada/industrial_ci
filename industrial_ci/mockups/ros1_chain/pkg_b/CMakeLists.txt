cmake_minimum_required(VERSION 3.0.2)
project(pkg_b)

find_package(catkin REQUIRED COMPONENTS
  pkg_a_underlay
)

catkin_package(
  CATKIN_DEPENDS pkg_a_underlay
)
