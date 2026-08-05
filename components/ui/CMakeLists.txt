idf_component_register(SRCS "mode_eye.cpp" "mode_radar.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES config types esp_lcd
                       PRIV_REQUIRES esp_timer)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
