idf_component_register(SRCS "radar_stack_single.cpp" "radar_util.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES config types drivers rtos esp_adc)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
