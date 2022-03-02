add_executable(offboard src/main.cpp)

target_include_directories(offboard PUBLIC include)


target_link_libraries(offboard PRIVATE
comm_lib
MAVSDK::mavsdk
fastcdr 
fastrtps
)
