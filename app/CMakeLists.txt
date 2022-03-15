add_executable(quad_control src/quad_control.cpp )



target_include_directories(quad_control PUBLIC include)


target_link_libraries(quad_control PRIVATE
Eigen3::Eigen
comm_lib
MAVSDK::mavsdk
yaml-cpp
fastcdr 
fastrtps
)
