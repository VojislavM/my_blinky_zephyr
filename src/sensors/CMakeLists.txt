zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lis2dw12_reg.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/sensors_izo.c)
