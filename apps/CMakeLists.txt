if(OMNI_BUILD_STEREO_VO)
  add_executable(stereo_vo
    ${CMAKE_SOURCE_DIR}/apps/vo/main.cpp
  )
  target_include_directories(stereo_vo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_vo omni_utils omni_device svo ${OpenCV_LIBS})
endif()
