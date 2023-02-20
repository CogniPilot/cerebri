# To ensure final path is absolute and does not contain ../.. in variable.
get_filename_component(APPLICATION_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)

# Add this project to list of board roots
#list(APPEND BOARD_ROOT ${APPLICATION_PROJECT_DIR})
