set(RADAR_SRC "radar_stack_single.cpp")

idf_component_register(SRCS ${RADAR_SRC} "radar_util.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES config types drivers rtos esp_adc)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
