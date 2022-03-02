add_executable(offboard src/main.cpp)

#target_include_directories(mocap_logger PUBLIC include)

target_link_libraries(offboard
    MAVSDK::mavsdk
)
# target_link_libraries(mocap_logger PRIVATE
# comm_lib
# )

# Third party dependencies
# target_link_libraries(mocap_logger PRIVATE
# fastcdr 
# fastrtps
# )