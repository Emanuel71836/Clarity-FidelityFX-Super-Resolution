include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

<<<<<<< HEAD
add_library(FSR_Clarity_Common INTERFACE)

set(config
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/FSR_Clarity.json
)

copyTargetCommand("${config}" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} copied_common_config)
add_dependencies(FSR_Clarity_Common copied_common_config)
=======
add_library(FSR2_Sample_Common INTERFACE)

set(config
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/FSR2_Sample.json
)

copyTargetCommand("${config}" ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} copied_common_config)
add_dependencies(FSR2_Sample_Common copied_common_config)
>>>>>>> 1680d1edd5c034f88ebbbb793d8b88f8842cf804
