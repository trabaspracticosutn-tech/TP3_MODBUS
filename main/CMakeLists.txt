configure_file("${CMAKE_CURRENT_LIST_DIR}/adquisición.c"
               "${CMAKE_CURRENT_BINARY_DIR}/adquisicion.c" COPYONLY)
idf_component_register(SRCS "main_comm.c" "comm.c" "Aplicacion.c"
                         "${CMAKE_CURRENT_BINARY_DIR}/adquisicion.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_adc esp_driver_dac esp_driver_gpio esp_driver_uart)

# idf.py -DAPP_MODBUS_ROLE=MASTER|SLAVE -DAPP_MODBUS_ADDRESS=1 build
set(APP_MODBUS_ROLE "MASTER" CACHE STRING "Rol Modbus: MASTER o SLAVE")
set_property(CACHE APP_MODBUS_ROLE PROPERTY STRINGS MASTER SLAVE)
set(APP_MODBUS_ADDRESS "1" CACHE STRING "Direccion del esclavo local o consultado (1..247)")
if(NOT APP_MODBUS_ROLE STREQUAL "MASTER" AND NOT APP_MODBUS_ROLE STREQUAL "SLAVE")
    message(FATAL_ERROR "APP_MODBUS_ROLE debe ser MASTER o SLAVE")
endif()
if(NOT APP_MODBUS_ADDRESS MATCHES "^[0-9]+$" OR
   APP_MODBUS_ADDRESS LESS 1 OR APP_MODBUS_ADDRESS GREATER 247)
    message(FATAL_ERROR "APP_MODBUS_ADDRESS debe estar entre 1 y 247")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    APP_MODBUS_ADDRESS=${APP_MODBUS_ADDRESS})
