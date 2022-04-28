add_executable(thrust_stand src/thrust_stand.cpp )



target_include_directories(thrust_stand PUBLIC include)


target_link_libraries(thrust_stand PRIVATE
Eigen3::Eigen
#comm_lib
MAVSDK::mavsdk
yaml-cpp
fastcdr 
fastrtps
)
