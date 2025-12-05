cmake_minimum_required(VERSION 3.10)

add_subdirectory(ffx-fsr2-api)
add_subdirectory(Common)

if(GFX_API STREQUAL "DX12")
    add_subdirectory(DX12)
elseif(GFX_API STREQUAL "VK")
    add_subdirectory(VK)
endif()
