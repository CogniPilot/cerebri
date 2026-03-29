target_sources(app PRIVATE nxp_flexio_dshot_api.c)

if(CONFIG_RDD2_DSHOT)
  target_sources(app PRIVATE nxp_flexio_dshot.c)
endif()

if(CONFIG_RDD2_SITL)
  target_sources(app PRIVATE sitl_dshot.c)
endif()
