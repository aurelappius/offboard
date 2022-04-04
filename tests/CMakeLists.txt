set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/gnuplot-iostream")

file(GLOB SRCS src/*.cpp)

add_executable(compensator_test compensator_test/main.cpp )

target_link_libraries(compensator_test ${Boost_LIBRARIES})