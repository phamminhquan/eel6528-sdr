cmake_minimum_required(VERSION 3.16.3)
project(sdr)

# Link packages
#get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
#include_directories(${JSON_INC_PATH})
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

# Include header directory
include_directories(include)

# Add source directory
file(GLOB SOURCES "src/*.cpp")

add_executable(test ${SOURCES})
target_link_libraries(test ${JSONCPP_LIBRARIES})


#cmake_minimum_required(VERSION 2.6.2)
#project(jsoncpp_test)

#find_package(jsoncpp REQUIRED)
#message("FOUND CONFIG: ${jsoncpp_CONFIG}")
#message("INCLUDE DIRS: ${jsoncpp_INCLUDE_DIRS}")
#message("LIBRARIES: ${jsoncpp_LIBRARIES}")
