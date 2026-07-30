# Default single-sensor backend. To restore pitch-stack fusion:
#   set(RADAR_SRC "radar_stack_multi.c")
# and restore the multi CFG_SENSORS block in config.h.
set(RADAR_SRC "radar_stack_single.c")

idf_component_register(SRCS "main.c" "display.c" "ld2450.c" ${RADAR_SRC}
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_lcd esp_lcd_gc9a01 esp_timer)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
