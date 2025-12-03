include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

add_library(FSR_Clarity_Common INTERFACE)

set(config
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/FSR_Clarity.json
)

copyTargetCommand("${config}" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} copied_common_config)
add_dependencies(FSR_Clarity_Common copied_common_config)
