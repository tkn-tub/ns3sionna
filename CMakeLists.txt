cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

# Check for stdint.h
check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

include(FindPkgConfig)
if(NOT PKG_CONFIG_FOUND)
    message(STATUS "pkgconf not found")
    return()
endif()

# Find ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)
include_directories(${ZeroMQ_INCLUDE_DIRS})


## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.h
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
)

## use the hint from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
)

#target_include_directories(sionna PRIVATE ${ZMQ_INCLUDE_DIRS})
#target_link_libraries(sionna PRIVATE ${ZMQ_LIBRARIES})

# protobuf lib
find_package(Protobuf REQUIRED)
if(NOT Protobuf_FOUND)
    message(STATUS "protobuf not found")
    return()
endif()

set(proto_source_files model/message.proto)

# Module sources and headers
set(SOURCE_FILES
        model/sionna-mobility-model.cc
        model/sionna-propagation-cache.cc
        model/sionna-propagation-delay-model.cc
        model/sionna-propagation-loss-model.cc
        model/sionna-spectrum-propagation-loss-model.cc
        model/sionna-phased-array-spectrum-propagation-loss-model.cc
        model/cfr-tag.cc
        helper/sionna-helper.cc
        ${proto_source_files}
)

set(HEADER_FILES
        model/sionna-mobility-model.h
        model/sionna-propagation-cache.h
        model/sionna-propagation-delay-model.h
        model/sionna-propagation-loss-model.h
        model/sionna-spectrum-propagation-loss-model.h
        model/sionna-phased-array-spectrum-propagation-loss-model.h
        model/cfr-tag.h
        helper/sionna-helper.h
        helper/sionna-utils.h
        #        ${PROTO_HDR}        # protobuf-generated header
)

# Examples (optional)
set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
            #test/sionna-examples-test-suite.cc
    )
endif()

# Build the ns-3 module
build_lib(
        LIBNAME sionna
        SOURCE_FILES ${SOURCE_FILES}
        HEADER_FILES ${HEADER_FILES}
        LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libpropagation}
        ${libspectrum}
        ${ZeroMQ_LIBRARIES}
        ${Protobuf_LIBRARIES}
)

# need protobuf_generate func to generate messages
check_function_exists(protobuf_generate protobuf_generate_exists)
if(${protobuf_generate_exists})
    message(STATUS "protobuf_generate command found")
else()
    message(STATUS "protobuf_generate command not found -> use a local copy from ${CMAKE_CURRENT_SOURCE_DIR}/protobuf-generate.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/protobuf-generate.cmake)
endif()

# generate proto for cpp
protobuf_generate(
        TARGET ${libsionna-obj}
        IMPORT_DIRS model/
        LANGUAGE cpp
        PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model
)

# generate proto for Python
protobuf_generate(
        TARGET ${libsionna-obj}
        IMPORT_DIRS model/
        LANGUAGE python
        PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model/ns3sionna/common
)
