file(GLOB APP_SOURCE_FILES
  real_main.c
  ${CMAKE_CURRENT_LIST_DIR}/app/src/*.c
  ${CMAKE_CURRENT_LIST_DIR}/app/src/common/*.c
  ${CMAKE_CURRENT_LIST_DIR}/app/src/pages/*.c
)

set(APP_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/app/src/
    ${CMAKE_CURRENT_LIST_DIR}/app/src/common
    ${CMAKE_CURRENT_LIST_DIR}/app/src/pages
)

message("APP_SOURCE_FILES: ${APP_SOURCE_FILES}")

idf_component_register(SRCS ${APP_SOURCE_FILES} INCLUDE_DIRS ${APP_INCLUDE_DIRS})
