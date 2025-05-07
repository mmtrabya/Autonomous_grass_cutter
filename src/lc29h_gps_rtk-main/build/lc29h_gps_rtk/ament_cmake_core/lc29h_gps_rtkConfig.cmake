# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lc29h_gps_rtk_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lc29h_gps_rtk_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lc29h_gps_rtk_FOUND FALSE)
  elseif(NOT lc29h_gps_rtk_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lc29h_gps_rtk_FOUND FALSE)
  endif()
  return()
endif()
set(_lc29h_gps_rtk_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lc29h_gps_rtk_FIND_QUIETLY)
  message(STATUS "Found lc29h_gps_rtk: 0.0.0 (${lc29h_gps_rtk_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lc29h_gps_rtk' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lc29h_gps_rtk_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lc29h_gps_rtk_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lc29h_gps_rtk_DIR}/${_extra}")
endforeach()
