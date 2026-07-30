idf_component_register(SRCS "main.c" "display.c" "ld2450.c" "radar_stack.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_lcd esp_lcd_gc9a01)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
