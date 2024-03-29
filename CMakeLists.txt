cmake_minimum_required(VERSION 3.1)
set(CMAKE_BUILD_TYPE debug)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 17)

include_directories(include)

include_directories(NH-ICBS-HLC/thirdparty/json/include)

include_directories(NH-ICBS-HLC/thirdparty/eigen)
include_directories(NH-ICBS-HLC/include)
add_subdirectory(NH-ICBS-HLC)

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

add_executable(main_hlc
    src/main_hlc.cpp
)

file(GLOB EXEC_SRC "experiments/*.cpp")
foreach(src_ ${EXEC_SRC})
	get_filename_component(target ${src_} NAME)
    
	string(REPLACE ".cpp" "" target ${target})
	string(REPLACE "/" "_" target ${target})
    message(STATUS "Found executable : ${target}")

	add_executable(${target} ${src_})
    target_link_libraries(${target} planner_lib cpm fastrtps fastcdr foonathan_memory)
endforeach()

target_link_libraries(main planner_lib cpm fastrtps fastcdr foonathan_memory)
target_link_libraries(main_hlc planner_lib cpm fastrtps fastcdr foonathan_memory)


