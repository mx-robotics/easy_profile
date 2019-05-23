add_library(easy_profile 
  EasyObjectDictionary.cpp
  EasyProfile.cpp
  EasyProtocol.cpp
)

install(TARGETS easy_profile
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)

