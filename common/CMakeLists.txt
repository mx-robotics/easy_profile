include_directories(include)
add_subdirectory(src)


install(DIRECTORY include/EasyProfile DESTINATION include
        FILES_MATCHING PATTERN "*.h")


if(EASYPROFILE_DEMOS_ENABLE)

find_package(Boost 1.40.0 REQUIRED system)

add_executable(demo_transducer_m main.cpp)
target_link_libraries(demo_transducer_m ${Boost_LIBRARIES} easy_profile)
install(TARGETS demo_transducer_m RUNTIME DESTINATION bin)

endif(EASYPROFILE_DEMOS_ENABLE)
