# FreeRTOS Based Library Integration for MbedUtils
# Targets are defined in: https://github.com/brandonbraun653/mbedutils
file(GLOB FREERTOS_INTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(mbedutils_lib_freertos STATIC ${FREERTOS_INTF_SOURCES})
target_link_libraries(mbedutils_lib_freertos PRIVATE
    mbedutils_headers
    mbedutils_internal_headers
    freertos_config  # Standard FreeRTOS integration target. Defined by the project for headers.
    etl::etl  # Embedded Template Library
)
