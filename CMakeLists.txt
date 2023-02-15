cmake_minimum_required(VERSION 3.8)

enable_language(CXX)

if (NOT DEFINED BOARD)
  set(BOARD native_posix_64)
endif()

set(ZEPHYR_MODULES
  ${CMAKE_SOURCE_DIR}/modules/sim_gz
  ${CMAKE_SOURCE_DIR}/modules/sim_core
  ${CMAKE_SOURCE_DIR}/modules/rover_control
  ${CMAKE_SOURCE_DIR}/modules/estimator
)

message(STATUS "ZEPHYR_MODULES: ${ZEPHYR_MODULES}")

set(ZephyrBuildConfiguration_ROOT cmake)

find_package(Zephyr 3.3.0 REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(cerebri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

zephyr_include_directories(include)

set(HCDF ${CMAKE_SOURCE_DIR}/../hcdformat/vehicles/mrb3s.hcdf)
add_custom_target(generate_hcdf ALL
	COMMAND ${CMAKE_SOURCE_DIR}/tools/cerebri_gen --clean --absolute --hcdf ${HCDF})

target_sources(app PRIVATE
  src/main.c
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS zephyr_pre1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
