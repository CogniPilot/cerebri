enable_language(CXX)

find_package(gz-transport12 REQUIRED)

zephyr_library()

zephyr_compile_options(
  -fexceptions
  -Wno-pedantic
  -Wno-missing-field-initializers
  -Wno-maybe-uninitialized
)

set(JINJA_FILES
  sim_gz_driver.cc
)

foreach(JINJA_FILE ${JINJA_FILES})
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/${JINJA_FILE}
    COMMAND ${JINJA_GEN}
    --hcdf ${HCDF_FILE}
    --template ${CMAKE_CURRENT_SOURCE_DIR}/src/${JINJA_FILE}.jinja
    --out ${CMAKE_CURRENT_BINARY_DIR}/src
    )
endforeach()

zephyr_include_directories(
  src
  ${CMAKE_CURRENT_BINARY_DIR}/src
  )

zephyr_library_sources(
  ${CMAKE_CURRENT_BINARY_DIR}/src/sim_gz_driver.cc
)

zephyr_library_include_directories(
  PRIVATE ${gz-transport12_INCLUDE_DIRS}
)

zephyr_library_link_libraries(
  PRIVATE ${gz-transport12_CORE_LIBRARY}
)

