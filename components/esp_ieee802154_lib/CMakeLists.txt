idf_component_register(
    INCLUDE_DIRS "include"
)


if(IDF_TARGET STREQUAL "esp32c6")
    set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32c6/lib802154.a")
else()
    message(FATAL_ERROR "Unsupported target for IEEE802.15.4 library: ${IDF_TARGET}")
endif()

target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIB_PATH})
