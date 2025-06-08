cmake_minimum_required(VERSION 3.0.2)
project(covins_comm)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        cv_bridge
        message_filters
        sensor_msgs
        nav_msgs
        geometry_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc) # imgcodecs for cv::imencode/imdecode

# Find Protocol Buffers
find_package(Protobuf REQUIRED)

# Cereal is already part of your thirdparty. Add its include path.
# IMPORTANT: Adjust this path to the directory that CONTAINS the "cereal" folder.
# For example, if cereal/archives/binary.hpp is at thirdparty/cereal/include/cereal/archives/binary.hpp
# then CEREAL_INCLUDE_DIR should be ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include
set(CEREAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal) # Default assumption
# If your Cereal headers are nested deeper, e.g., thirdparty/cereal/some_other_dir/cereal/
# then it might need to be: set(CEREAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/some_other_dir)

if(NOT EXISTS "${CEREAL_INCLUDE_DIR}/cereal/archives/binary.hpp") # More robust check for Cereal headers
    message(WARNING "Cereal include directory MAY be incorrect or not found at ${CEREAL_INCLUDE_DIR}/cereal/archives/binary.hpp. Please check path.")
    # Consider making this a FATAL_ERROR if you want to strictly enforce Cereal presence.
endif()


## Generate C++ source and header files from .proto files
## The `proto` directory should be at the root of `covins_comm`
set(PROTO_SRCS
        proto/data_buffer.proto
)
protobuf_generate_cpp(PROTO_GENERATED_SRCS PROTO_GENERATED_HDRS ${PROTO_SRCS})

source_group("Generated Files" FILES ${PROTO_GENERATED_SRCS} ${PROTO_GENERATED_HDRS})


## Declare a C++ library that covers all our abstraction and concrete implementations
add_library(covins_comm_lib_abstract
        src/comm_abstraction/CommunicatorFactory.cpp
        src/comm_messages/MsgImage.cpp
        src/comm_messages/MsgKeyframe.cpp
        src/comm_messages/MsgLandmark.cpp
        src/comm_messages/MsgOdometry.cpp
        src/comm_protocols/Ros1Communicator.cpp
        src/comm_protocols/TcpSocketCommunicator.cpp
        src/comm_serialization/CerealSerializer.cpp
        src/comm_serialization/ProtobufSerializer.cpp
        ${PROTO_GENERATED_SRCS} # Include the generated .cpp file from Protobuf
)

## Specify libraries to link a library or executable against
target_link_libraries(covins_comm_lib_abstract
        ${catkin_LIBRARIES}
        ${Eigen3_LIBS}
        ${OpenCV_LIBS}
        ${PROTOBUF_LIBRARIES} # Link against Protobuf
)

## Specify folders to look for headers in
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIRS} # Protobuf generated headers
        ${CEREAL_INCLUDE_DIR}   # Include path for your vendored Cereal library

        # IMPORTANT: Add include paths for other covins packages that this one depends on
        # Assuming covins_backend/include or covins_backend/include/covins is where TypeDefs.h lives
        # You might need to adjust this path based on your actual covins_backend installation/source location
        /root/covins_ws/src/covins/covins_backend/include # Common pattern for sibling packages
        # If covins_backend is installed, it might be in ${CATKIN_PACKAGE_INCLUDE_DESTINATION} or similar
        # Or, if covins_backend is itself a Catkin package, you might need to find_package it.
        # For now, this direct path is a common workaround for interconnected source trees.
)

# Add the directory containing generated protobuf headers to include paths
# These are generated in the build directory.
include_directories(${CMAKE_CURRENT_BINARY_DIR})


## IMPORTANT: Call catkin_package() BEFORE any install() commands that use
## Catkin-defined destination variables like CATKIN_PACKAGE_LIB_DESTINATION.
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES covins_comm_lib_abstract
        CATKIN_DEPENDS roscpp image_transport cv_bridge message_filters sensor_msgs nav_msgs geometry_msgs
        # No direct DEPENDS for Eigen3, OpenCV, protobuf here; handled by find_package and target_link_libraries
)


## Install project-specific headers and libraries
# Install our new header directories under covins/
install(DIRECTORY include/covins/comm_abstraction/
        include/covins/comm_messages/
        include/covins/comm_protocols/
        include/covins/comm_serialization/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/covins
)

# Install the library we just built
install(TARGETS covins_comm_lib_abstract
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install generated Protobuf headers (important for other packages to find them)
install(FILES ${PROTO_GENERATED_HDRS}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/covins/protobuf_generated
)

# Any other targets/installs your original covins_comm had should be retained.
