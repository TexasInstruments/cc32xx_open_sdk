#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "NoRtos::nortos_cc32xx" for configuration "Release"
set_property(TARGET NoRtos::nortos_cc32xx APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(NoRtos::nortos_cc32xx PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/kernel/nortos/lib/gcc/m4/nortos_cc32xx.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS NoRtos::nortos_cc32xx )
list(APPEND _IMPORT_CHECK_FILES_FOR_NoRtos::nortos_cc32xx "${_IMPORT_PREFIX}/kernel/nortos/lib/gcc/m4/nortos_cc32xx.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
