if(CONFIG_CEREBRI_SITL)
  target_sources(app PRIVATE sitl_imu.c)
endif()
