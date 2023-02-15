find_package(gz-transport12 REQUIRED)

zephyr_include_directories(.)

zephyr_library()

zephyr_compile_options(
  -fexceptions
  -Wno-pedantic
  -Wno-missing-field-initializers
  -Wno-maybe-uninitialized
)

zephyr_library_sources(
  src/sim_gz_driver.cc
)

zephyr_library_include_directories(
  PRIVATE ${gz-transport12_INCLUDE_DIRS}
)

zephyr_library_link_libraries(
  PRIVATE ${gz-transport12_CORE_LIBRARY}
)

