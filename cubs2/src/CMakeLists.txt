target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  control_io.c
  generated_fixed_wing/CubControl_FixedWingOuterLoop.c
  main.c
  topic_flatbuffer.c
  topic_shell.c
)

if(CONFIG_CODE_DATA_RELOCATION)
  set(RELOCATE_FILES
    main.c
    control_io.c
    topic_flatbuffer.c
    generated_fixed_wing/CubControl_FixedWingOuterLoop.c
  )

  if(CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064 OR CONFIG_BOARD_VMU_RT1170_MIMXRT1176_CM7)
    zephyr_code_relocate(
      FILES ${RELOCATE_FILES}
      LOCATION ITCM_TEXT_RODATA
    )

    zephyr_code_relocate(
      FILES topic_shell.c
      FILTER ".*(flight_state_store_publish|motor_output_store_publish|cubs2_topic_flight_state_publish|cubs2_topic_motor_output_publish).*"
      LOCATION ITCM_TEXT
    )
  endif()
endif()
