enable_language(CXX)

zephyr_include_directories(.)

zephyr_library()

zephyr_compile_options(-Wno-unused-parameter)

zephyr_library_sources(
    src/sim_core.cc
    src/channels.c
)

