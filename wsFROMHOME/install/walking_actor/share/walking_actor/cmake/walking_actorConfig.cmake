# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_walking_actor_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED walking_actor_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(walking_actor_FOUND FALSE)
  elseif(NOT walking_actor_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(walking_actor_FOUND FALSE)
  endif()
  return()
endif()
set(_walking_actor_CONFIG_INCLUDED TRUE)

# output package information
if(NOT walking_actor_FIND_QUIETLY)
  message(STATUS "Found walking_actor: 0.221.2 (${walking_actor_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'walking_actor' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${walking_actor_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(walking_actor_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${walking_actor_DIR}/${_extra}")
endforeach()
