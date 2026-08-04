idf_component_register(SRCS "display.cpp" "ld2450.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES config driver esp_lcd
                       PRIV_REQUIRES esp_lcd_gc9a01)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
