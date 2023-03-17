if (CONFIG_TINYFRAME)

set(TINYFRAME_DIR ${ZEPHYR_CURRENT_MODULE_DIR})

zephyr_library()

zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${TINYFRAME_DIR}
  )

zephyr_library_sources(
  ${TINYFRAME_DIR}/TinyFrame.c
  ${TINYFRAME_DIR}/demo/utils.c
)

endif()
