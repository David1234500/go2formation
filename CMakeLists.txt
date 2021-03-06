cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 17)

include_directories(SimpleHLC/thirdparty/eigen)
include_directories(SimpleHLC/include)
add_subdirectory(SimpleHLC)

# Fast DDS / eProsima
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")


# CPM lib
include_directories(SYSTEM ../cpm_lib/include/)
include_directories(SYSTEM ../cpm_lib/include/cpm/dds/)
link_directories(../cpm_lib/build/)


include_directories(src)
link_libraries(dl nsl m pthread rt)

add_executable(main
    src/main.cpp
)
target_link_libraries(main planner_lib cpm fastrtps fastcdr foonathan_memory)


