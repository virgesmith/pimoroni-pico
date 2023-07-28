set(MOD_NAME bsec)
string(TOUPPER ${MOD_NAME} MOD_NAME_UPPER)
add_library(usermod_${MOD_NAME} INTERFACE)


target_sources(usermod_${MOD_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/${MOD_NAME}_adapter.c
    ${CMAKE_CURRENT_LIST_DIR}/${MOD_NAME}_adapter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/${MOD_NAME}.cpp
)

target_include_directories(usermod_${MOD_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_${MOD_NAME} INTERFACE
    MODULE_${MOD_NAME_UPPER}_ENABLED=1
)

add_library(algobsec STATIC IMPORTED)
set_target_properties(algobsec PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libalgobsec.a)

target_link_libraries(usermod INTERFACE usermod_${MOD_NAME}
    bme68x
    algobsec
)