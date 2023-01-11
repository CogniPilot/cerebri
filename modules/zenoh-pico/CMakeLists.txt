if (CONFIG_ZENOH_PICO)

set(ZENOH_PICO_DIR ${ZEPHYR_CURRENT_MODULE_DIR})

zephyr_library()

zephyr_library_compile_definitions(
  ZENOH_ZEPHYR
  ZENOH_DEBUG=3
  Z_MULTI_THREAD=1
  Z_LINK_TCP=1
  Z_LINK_UDP_MULTICAST=1
  Z_LINK_UDP_UNICAST=1
  Z_LINK_BLUETOOTH=0
  Z_LINK_SERIAL=0
  Z_SCOUTING_UDP=1
  )

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/include/zephyr
  )

zephyr_include_directories(
  ${ZENOH_PICO_DIR}/zenoh-pico/include
  )


file(GLOB_RECURSE SRCS
  ${ZENOH_PICO_DIR}/zenoh-pico/src/api/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/collections/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/link/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/net/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/protocol/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/session/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/protocol/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/system/zephyr/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/transport/*
  ${ZENOH_PICO_DIR}/zenoh-pico/src/utils/*
  )

zephyr_library_sources(
  ${SRCS}
)

endif(CONFIG_ZENOH_PICO)