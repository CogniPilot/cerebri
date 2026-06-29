target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

if(CONFIG_CUBS2_CSYN)
  target_sources(app PRIVATE
    csyn.c
  )

  target_sources_ifdef(CONFIG_CUBS2_CSYN_SHELL app PRIVATE
    csyn_shell.c
  )

  target_sources_ifdef(CONFIG_CUBS2_CSYN_NATIVE_UDP app PRIVATE
    csyn_native_udp.c
  )
endif()
