zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources(
    src/sim_core.cc
    src/channels.c
)

#zephyr_library_include_directories()

#zephyr_library_link_libraries()
