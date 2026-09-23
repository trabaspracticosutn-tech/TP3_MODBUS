configure_file("${CMAKE_CURRENT_LIST_DIR}/adquisición.c"
               "${CMAKE_CURRENT_BINARY_DIR}/adquisicion.c" COPYONLY)
idf_component_register(SRCS "main_comm.c" "comm.c" "Aplicacion.c"
                         "${CMAKE_CURRENT_BINARY_DIR}/adquisicion.c"
                    INCLUDE_DIRS ".")
