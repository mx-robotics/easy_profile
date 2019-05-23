cmake_minimum_required(VERSION 2.6)
project(transducer_m)

find_package(Boost 1.40.0 REQUIRED system)
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(common)
add_executable(transducer_m main.cpp)
target_link_libraries(transducer_m ${Boost_LIBRARIES} easy_profile)
install(TARGETS transducer_m RUNTIME DESTINATION bin)
