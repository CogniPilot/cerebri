zephyr_library()

zephyr_compile_options(-Wno-unused-parameter)

set(JINJA_FILES
  mixer.c
  parameters.h
)

foreach(JINJA_FILE ${JINJA_FILES})
  message(STATUS "jinja file: ${JINJA_FILE}")
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/${JINJA_FILE}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${JINJA_FILE}.jinja
    COMMAND ${JINJA_GEN} --hcdf ${HCDF_FILE}
    --template ${CMAKE_CURRENT_SOURCE_DIR}/src/${JINJA_FILE}.jinja
    --out ${CMAKE_CURRENT_BINARY_DIR}/src
    )
endforeach()

zephyr_include_directories(
  src
  ${CMAKE_CURRENT_BINARY_DIR}/src
  )

zephyr_library_sources(
  #src/controller.c
  #src/casadi/multicopter.c
  #${CMAKE_CURRENT_BINARY_DIR}/src/mixer.c
  #${CMAKE_CURRENT_BINARY_DIR}/src/parameters.h
)

