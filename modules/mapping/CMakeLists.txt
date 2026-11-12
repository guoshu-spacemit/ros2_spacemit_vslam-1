cmake_minimum_required(VERSION 3.22)
project(spacemit_mapping LANGUAGES NONE)

include("${CMAKE_CURRENT_LIST_DIR}/../../tools/binary_sdk.cmake")
spacemit_install_binary_sdk(spacemit_mapping "../..")

install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")
