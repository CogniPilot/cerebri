target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  attitude_control.c
  attitude_estimator.c
  control_io.c
  fc_shell.c
  flight_mode.c
  imu_stream.c
  imu_latency_stats.c
  main.c
  motor_output.c
  motor_shell.c
  pid_axis.c
  rc_input.c
  rate_control.c
  top_shell.c
  topic_bus.c
  topic_flatbuffer.c
  topic_shell.c
)

if(CONFIG_CODE_DATA_RELOCATION AND CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064)
  zephyr_code_relocate(
    FILES
      main.c
      control_io.c
      imu_stream.c
      rc_input.c
      flight_mode.c
      attitude_estimator.c
      attitude_control.c
      rate_control.c
      motor_output.c
      imu_latency_stats.c
      topic_flatbuffer.c
      pid_axis.c
    LOCATION ITCM_TEXT_RODATA
  )

  zephyr_code_relocate(
    FILES
      imu_stream.c
      rc_input.c
    LOCATION DTCM_DATA_BSS
  )
endif()
