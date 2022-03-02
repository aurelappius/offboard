cmake_minimum_required(VERSION 3.10.2)
project(offboard VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(MAVSDK REQUIRED)


add_subdirectory(app)
add_subdirectory(lib)