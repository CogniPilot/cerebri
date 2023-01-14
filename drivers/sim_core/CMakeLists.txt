if (CONFIG_CEREBRI_SIM_CORE)

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources(
    src/sim_core.cc
)

#zephyr_library_include_directories()

#zephyr_library_link_libraries()

endif(CONFIG_CEREBRI_SIM_CORE)