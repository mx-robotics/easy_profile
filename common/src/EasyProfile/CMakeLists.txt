#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(easy_profile
  EasyObjectDictionary.cpp
  EasyProfile.cpp
  EasyProtocol.cpp
)

install(TARGETS easy_profile
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION usr/lib
        ARCHIVE DESTINATION lib/static)

