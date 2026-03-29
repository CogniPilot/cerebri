if(CONFIG_RDD2_SITL)
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

  target_sources(app PRIVATE
    sitl_flatbuffer.c
    sitl_udp_coordinator.c
  )
endif()
