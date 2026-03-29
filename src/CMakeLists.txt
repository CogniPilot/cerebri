target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  control_io.c
  fc_shell.c
  imu_stream.c
  main.c
  motor_output.c
  motor_shell.c
  rc_input.c
  rate_control.c
  topic_flatbuffer.c
  topic_shell.c
)
