include_directories(include)
add_subdirectory(src)


install(DIRECTORY include/EasyProfile DESTINATION include
        FILES_MATCHING PATTERN "*.h")


if(EASYPROFILE_DEMOS_ENABLE)

find_package(Boost 1.40.0 REQUIRED system)

add_executable(easy_profile_demo main.cpp)
target_link_libraries(easy_profile_demo ${Boost_LIBRARIES} easy_profile)
install(TARGETS easy_profile_demo RUNTIME DESTINATION bin)

endif(EASYPROFILE_DEMOS_ENABLE)
