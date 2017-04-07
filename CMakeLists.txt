cmake_minimum_required(VERSION 2.8.7)

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

if(APPLE)
elseif(UNIX)
  set(NOCDE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -pthread")
endif()

if (NCODE_NET_DEBUG)
  set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -O0 -fno-omit-frame-pointer --coverage")
else()
  set(NCODE_NET_BASE_FLAGS "${NCODE_NET_BASE_FLAGS} -O3 -march=native -DNDEBUG")   
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NCODE_NET_BASE_FLAGS}")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${NCODE_NET_BASE_LD_FLAGS} --coverage")

if (NOT NCODE_NET_DISABLE_TESTS)
   include(CTest)
   add_subdirectory(external/googletest)
   macro(add_test_exec name src_file deps)
     add_executable(${name} ${src_file})
     target_link_libraries(${name} gtest gmock_main ${deps} ${ARGN})
     add_test(NAME ${name} COMMAND ${name})
   endmacro(add_test_exec)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions)
find_package(Protobuf REQUIRED)
find_package(PCAP REQUIRED)
find_package(NCodeCommon REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS} ${PCAP_INCLUDE_DIR} ${CMAKE_BINARY_DIR})

################################
# Network-releated stuff
################################
PROTOBUF_GENERATE_CPP(PROTO_NET_SRCS PROTO_NET_HDRS src/net.proto)
set_property(SOURCE ${PROTO_NET_SRCS} APPEND_STRING PROPERTY COMPILE_FLAGS "-Wno-extended-offsetof")
set(NET_HEADER_FILES src/net_common.h src/net_gen.h src/pcap.h src/algorithm.h src/trie.h ${PROTO_NET_HDRS})
add_library(ncode_net SHARED src/net_common.cc src/net_gen.cc src/pcap.cc src/algorithm.cc ${PROTO_NET_SRCS} ${NET_HEADER_FILES})
target_link_libraries(ncode_net ${PCAP_LIBRARY} ${PROTOBUF_LIBRARIES} ncode_common)

if (NOT NCODE_NET_DISABLE_TESTS)
  add_test_exec(net_common_test src/net_common_test.cc ncode_net)
  add_test_exec(net_gen_test src/net_gen_test.cc ncode_net)
  add_test_exec(net_algorithm_test src/algorithm_test.cc ncode_net)
  add_test_exec(net_trie_test src/trie_test.cc ncode_net)
#  add_test_exec(net_constraint_test src/constraint_test.cc ncode_net)
#  add_test_exec(net_path_cache_test src/path_cache_test.cc ncode_net)
endif()

if (NOT NCODE_NET_DISABLE_BENCHMARKS)
  add_executable(net_algorithm_benchmark src/algorithm_benchmark.cc)
  target_link_libraries(net_algorithm_benchmark ncode_net)
endif()

INSTALL(TARGETS ncode_net
  DESTINATION "lib")

INSTALL (
  FILES ${NET_HEADER_FILES}
  DESTINATION include/ncode/ncode_net)
