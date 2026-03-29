target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  attitude_control.c
  attitude_estimator.c
  control_io.c
  fc_shell.c
  flight_mode.c
  generated/PIDAxis.c
  generated/rdd2.c
  imu_stream.c
  main.c
  motor_output.c
  motor_shell.c
  rc_input.c
  rate_control.c
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
      topic_flatbuffer.c
      generated/PIDAxis.c
      generated/rdd2.c
    LOCATION ITCM_TEXT_RODATA
  )

  zephyr_code_relocate(
    FILES
      topic_shell.c
    FILTER ".*(flight_state_store_publish|motor_output_store_publish|rdd2_topic_flight_state_publish|rdd2_topic_motor_output_publish).*"
    LOCATION ITCM_TEXT
  )

  zephyr_code_relocate(
    FILES
      imu_stream.c
      rc_input.c
    LOCATION DTCM_DATA_BSS
  )
endif()
