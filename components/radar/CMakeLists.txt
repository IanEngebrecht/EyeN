# Default single-sensor backend. To restore pitch-stack fusion:
#   set(RADAR_SRC "radar_stack_multi.cpp")
# and restore the multi CFG_SENSORS block in config.h.
set(RADAR_SRC "radar_stack_single.cpp")

idf_component_register(SRCS ${RADAR_SRC}
                       INCLUDE_DIRS "include"
                       REQUIRES config drivers esp_adc)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
