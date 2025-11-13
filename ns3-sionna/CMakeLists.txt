# Find Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

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

# Add custom ns-3 models to a sionna library
add_library(
        sionna-lib
        lib/message.pb.cc
        lib/sionna-utils.h
        lib/sionna-helper.cc
        lib/sionna-mobility-model.cc
        lib/sionna-propagation-cache.cc
        lib/sionna-propagation-delay-model.cc
        lib/sionna-propagation-loss-model.cc
        lib/sionna-spectrum-propagation-loss-model.cc
        lib/sionna-phased-array-spectrum-propagation-loss-model.cc
        lib/cfr-tag.cc
)

# Link sionna library with ZeroMQ and Protobuf
target_link_libraries(sionna-lib ${libwifi} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES})

# Example skripts
build_exec(
        EXECNAME example-ns3
        SOURCE_FILES example-ns3.cc
        LIBRARIES_TO_LINK ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-sionna
        SOURCE_FILES example-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-mobility-sionna
        SOURCE_FILES example-mobility-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-munich-sionna
        SOURCE_FILES example-munich-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-munich-mobility-sionna
        SOURCE_FILES example-munich-mobility-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-sionna-spectrummodel
        SOURCE_FILES example-sionna-spectrummodel.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME performance-ns3
        SOURCE_FILES performance-ns3.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME performance-sionna
        SOURCE_FILES performance-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-optimized-sionna
        SOURCE_FILES example-optimized-sionna.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-sionna-snr
        SOURCE_FILES example-sionna-snr.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-sionna-sensing
        SOURCE_FILES example-sionna-sensing.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)

build_exec(
        EXECNAME example-sionna-sensing-mobile
        SOURCE_FILES example-sionna-sensing-mobile.cc
        LIBRARIES_TO_LINK sionna-lib ${libcore} ${ns3-libs} ${Protobuf_LIBRARIES} ${ZeroMQ_LIBRARIES}
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/ns3-sionna
)