
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was spacemit_mappingConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################
set(spacemit_mapping_FOUND TRUE)
set(spacemit_mapping_functional_FOUND TRUE)
set(spacemit_mapping_TARGET_SYSTEM "Linux")
set(spacemit_mapping_TARGET_PROCESSOR "riscv64")

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _spacemit_mapping_consumer_processor)
if(NOT CMAKE_SYSTEM_NAME STREQUAL spacemit_mapping_TARGET_SYSTEM OR
   NOT _spacemit_mapping_consumer_processor STREQUAL spacemit_mapping_TARGET_PROCESSOR)
  set(spacemit_mapping_FOUND FALSE)
  set(spacemit_mapping_NOT_FOUND_MESSAGE
    "SDK target ${spacemit_mapping_TARGET_SYSTEM}/${spacemit_mapping_TARGET_PROCESSOR} does not match the consumer toolchain")
  return()
endif()

check_required_components(spacemit_mapping)
if(NOT spacemit_mapping_FOUND)
  return()
endif()

set_and_check(spacemit_mapping_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set_and_check(spacemit_mapping_PUBLIC_HEADER
  "${spacemit_mapping_INCLUDE_DIR}/spacemit_mapping.h")
include("${CMAKE_CURRENT_LIST_DIR}/spacemit_mappingTargets.cmake")
