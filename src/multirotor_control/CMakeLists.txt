zephyr_library()

zephyr_compile_options(-Wno-unused-parameter)

set(JINJA_FILES
  controller.c
)

foreach(JINJA_FILE JINJA_FILES)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/${JINJA_FILE}
    COMMAND ${JINJA_GEN} --hcdf ${HCDF_FILE} --file
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${JINJA_FILE}.jinja
    )
endforeach()

zephyr_include_directories(
  src
  ${CMAKE_CURRENT_BINARY_DIR}/src
  )

zephyr_library_sources(
  ${CMAKE_CURRENT_BINARY_DIR}/src/controller.c
)

