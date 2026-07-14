# Per-target component dependencies — the I2C master driver lives in a different
# component per SDK: "esp8266" on ESP8266_RTOS_SDK, "driver" on ESP-IDF (v4/v5).
# IDF_TARGET is a build property, not a plain variable, so it must be read via
# idf_build_get_property (a bare ${IDF_TARGET} is empty here).
idf_build_get_property(_ade_target IDF_TARGET)
if(_ade_target STREQUAL "esp8266")
    idf_component_register(SRCS "ade7953.c" "ade7953_i2c.c"
                           INCLUDE_DIRS "."
                           REQUIRES esp8266)
else()
    idf_component_register(SRCS "ade7953.c" "ade7953_i2c.c"
                           INCLUDE_DIRS "."
                           REQUIRES driver)
endif()
