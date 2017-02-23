# - Try to find ncode
#
# Usage of this module as follows:
#
#     find_package(NCodeCommon)
#
# Variables defined by this module:
#
#  NCODE_COMMON_FOUND        Whether or not ncode-common was found
#  NCODE_COMMON_INCLUDE_DIR  The ncode-common include directory.
#  NCODE_COMMON_LIBRARY      The ncode-common library

find_path(NCODE_COMMON_INCLUDE_DIR
    NAMES ncode/ncode_common/common.h)

find_library(NCODE_COMMON_LIBRARY
  NAMES ncode_common)

message("NCodeCommon include dir = ${NCODE_COMMON_INCLUDE_DIR}")
message("NCodeCommon lib = ${NCODE_COMMON_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NCodeCommon DEFAULT_MSG
    NCODE_COMMON_LIBRARY
    NCODE_COMMON_INCLUDE_DIR)
  
mark_as_advanced(
    NCODE_COMMON_INCLUDE_DIR
    NCODE_COMMON_LIBRARY)
