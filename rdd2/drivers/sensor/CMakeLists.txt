if(CONFIG_RDD2_SITL)
  target_sources(app PRIVATE sitl_imu.c)
endif()
