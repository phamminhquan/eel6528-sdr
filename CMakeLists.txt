cmake_minimum_required(VERSION 3.16.3)
project(sdr)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# set gdb flag
set(CMAKE_CXX_FLAGS "-g")

# set optmization flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Add source directory
file(GLOB SOURCES "src/*.cpp" "include/*.cpp")

# Link VOLK
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_VOLK")

# Link pthread package
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(Threads REQUIRED)

# Link jsoncpp package
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(JSONCPP jsoncpp)
#link_libraries(${JSONCPP_LIBRARIES})

# Link uhd and boost package
find_package(UHD 3.15.0 REQUIRED)
set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
set(BOOST_MIN_VERSION 1.71)
include(UHDBoost)

# need these include and link directories for the build
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})


# Include header directory
include_directories(include)
include_directories(/usr/include)
link_directories(/usr/lib/x86_64-linux-gnu)

add_executable(test ${SOURCES})

# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
#message(STATUS "Linking shared jsoncpp library.")
#target_link_libraries(test ${JSONCPP_LIBRARIES})
message(STATUS "Linking shared UHD library.")
target_link_libraries(test ${UHD_LIBRARIES} ${Boost_LIBRARIES})
message(STATUS "Linking shared pthread library.")
target_link_libraries(test ${CMAKE_THREAD_LIBS_INT})
message(STATUS "Linking shared FFTW3 library.")
target_link_libraries(test fftw3f fftw3f_threads volk)
