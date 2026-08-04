idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES config drivers radar ui
                                driver esp_timer)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
