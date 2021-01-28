cmake_minimum_required(VERSION 3.5)

## Project branding, version and package mantainer
project(soccerSim)
set(VERSION "0.9")
set(MAINTAINER "Erfan Fathi <fathierfan97@gmail.com>")


# some utils and helper vars
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

set(app ${CMAKE_PROJECT_NAME})
# create the target before the sources list is known so that we can call
add_executable(${app} "")


# set explicitly the c++ standard to use
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add src dir to included directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/physics)


## Handling depenendcies
# we will append all libs to this var
set(libs)

# Qt
find_package(Qt5 COMPONENTS Network REQUIRED)
list(APPEND libs Qt5::Network)

# ODE
find_package(ODE REQUIRED)
list(APPEND libs ode::ode)

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
list(APPEND libs ${PROTOBUF_LIBRARIES})

protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/proto/observation.proto
    src/proto/commands.proto
)

set(SOURCES
    src/main.cpp
    src/physics/pworld.cpp
    src/physics/pobject.cpp
    src/physics/pball.cpp
    src/physics/pground.cpp
    src/physics/pfixedbox.cpp
    src/physics/pcylinder.cpp
    src/physics/pbox.cpp
    src/physics/pray.cpp
    src/sslworld.cpp
    src/robot.cpp
)

set(HEADERS
    include/physics/pworld.h
    include/physics/pobject.h
    include/physics/pball.h
    include/physics/pground.h
    include/physics/pfixedbox.h
    include/physics/pcylinder.h
    include/physics/pbox.h
    include/physics/pray.h
    include/sslworld.h
    include/robot.h
    config/config.h
)

# files to be compiled
set(srcs
    ${PROTO_CPP}
    ${PROTO_H}
    ${HEADERS}
    ${SOURCES}
)

target_sources(${app} PRIVATE ${srcs})
install(TARGETS ${app} DESTINATION bin)
target_link_libraries(${app} ${libs})