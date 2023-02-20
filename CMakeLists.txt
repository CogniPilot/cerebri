cmake_minimum_required(VERSION 3.8)

enable_language(CXX)

if (NOT DEFINED BOARD)
  set(BOARD native_posix_64)
endif()

set(ZEPHYR_EXTRA_MODULES
  ${CMAKE_SOURCE_DIR}/src
)

find_package(hcdformat REQUIRED)
find_package(ament_cmake REQUIRED)

# find dependencies
if (NOT DEFINED HCDF)
  set(HCDF mrb3s)
endif()
set(HCDF_FILE "${hcdformat_DIR}/../vehicles/${HCDF}.hcdf")
set(JINJA_GEN "${CMAKE_SOURCE_DIR}/tools/cerebri_gen")
message(STATUS "Jinja gen: ${JINJA_GEN}")

set(ZephyrBuildConfiguration_ROOT cmake)
find_package(Zephyr 3.2.0 REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(cerebri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/zephyr/zephyr.elf
  DESTINATION lib/${PROJECT_NAME}
  RENAME cerebri
)

ament_package()
