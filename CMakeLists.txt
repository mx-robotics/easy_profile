cmake_minimum_required(VERSION 2.6)
project(EasyProfile)

include_directories(common/include/)
add_subdirectory(common)


option(EASYPROFILE_DEMOS_ENABLE "Builds EasyProfile transducerM demos" ON)
