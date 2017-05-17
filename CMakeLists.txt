cmake_minimum_required(VERSION 2.8.7)

# If there is already a target called ncode_net someone has already
# added this project. Will ignore.
if(TARGET ncode_net)
  return()
endif(TARGET ncode_net)

project(NCodeNet)
option(NCODE_NET_DISABLE_TESTS "If tests should be compiled or not" ON)
option(NCODE_NET_DISABLE_BENCHMARKS "If benchmarks should be compiled or not" ON)
option(NCODE_NET_DEBUG "A debug build" OFF)
option(NCODE_NET_ASAN "Compile with ASAN on" OFF)
option(NCODE_NET_TSAN "Compile with TSAN on" OFF)

set(NCODE_NET_BASE_FLAGS "-g -std=c++11 -pedantic-errors -Winit-self -Woverloaded-virtual -Wuninitialized -Wall -Wextra -fno-exceptions")
set(NCODE_NET_BASE_LD_FLAGS "")
if (NCODE_NET_ASAN)
   set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(NCODE_NET_BASE_LD_FLAGS "${NCODE_NET_BASE_LD_FLAGS} -fsanitize=address")
endif()
if (NCODE_NET_TSAN)
   set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(NCODE_NET_BASE_LD_FLAGS "${NCODE_NET_BASE_LD_FLAGS} -fsanitize=thread")
endif()

if (NCODE_NET_DEBUG)
  set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -O0 -fno-omit-frame-pointer")
else()
  set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -O3 -march=native -DNDEBUG")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NCODE_NET_BASE_FLAGS}")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${NCODE_NET_BASE_LD_FLAGS}")

if (NOT NCODE_NET_DISABLE_TESTS)
   include(CTest)
   add_subdirectory(external/googletest)
   macro(add_test_exec name src_file deps)
     add_executable(${name} ${src_file})
     target_link_libraries(${name} gtest gmock_main ${deps} ${ARGN})
     add_test(NAME ${name} COMMAND ${name})
   endmacro(add_test_exec)
endif()

add_library(ncode_common STATIC IMPORTED)
set_property(TARGET ncode_common PROPERTY IMPORTED_LOCATION "${CMAKE_BINARY_DIR}/external/lib/libncode_common.a")
if (NOT EXISTS ${CMAKE_BINARY_DIR}/external/lib/libncode_common.a)
  MESSAGE(STATUS "Unable to find ${CMAKE_BINARY_DIR}/external/lib/libncode_common.a")
  include(ExternalProject)
  ExternalProject_Add(ncode_common_lib
                      PREFIX ${CMAKE_BINARY_DIR}/ncode_common
                      GIT_REPOSITORY "https://github.com/ngvozdiev/ncode-common.git"
                      GIT_TAG "master"
                      BINARY_DIR ${CMAKE_BINARY_DIR}/ext_build/ncode_common
                      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/external
                      BUILD_COMMAND make)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions)
find_package(PCAP REQUIRED)
include_directories(${PCAP_INCLUDE_DIR} ${CMAKE_BINARY_DIR}/external/include)

set(NET_HEADER_FILES src/net_common.h src/net_gen.h src/pcap.h src/algorithm.h src/trie.h src/graph_query.h)
add_library(ncode_net STATIC src/net_common.cc src/net_gen.cc src/pcap.cc src/algorithm.cc src/graph_query.cc ${NET_HEADER_FILES})
target_link_libraries(ncode_net ${PCAP_LIBRARY} ncode_common)

if (NOT NCODE_NET_DISABLE_TESTS)
  add_test_exec(net_common_test src/net_common_test.cc ncode_net)
  add_test_exec(net_gen_test src/net_gen_test.cc ncode_net)
  add_test_exec(net_algorithm_test src/algorithm_test.cc ncode_net)
  add_test_exec(net_trie_test src/trie_test.cc ncode_net)
  add_test_exec(net_graph_query_test src/graph_query_test.cc ncode_net)
endif()

if (NOT NCODE_NET_DISABLE_BENCHMARKS)
  add_executable(net_algorithm_benchmark src/algorithm_benchmark.cc)
  target_link_libraries(net_algorithm_benchmark ncode_net)
endif()

install (TARGETS ncode_net DESTINATION lib)
install (FILES ${NET_HEADER_FILES} DESTINATION include/ncode_common)
