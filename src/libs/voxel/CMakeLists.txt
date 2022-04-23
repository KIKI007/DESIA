cmake_minimum_required(VERSION 3.5)
project(voxel)

set(CRL_TARGET_NAME ${PROJECT_NAME})

find_package(Boost REQUIRED)

file(GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/voxel/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
        "${Boost_LIBRARIES}"
        "igl::core"
)

message(${Boost_INCLUDE_DIRS})

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
        PUBLIC "${Boost_INCLUDE_DIRS}"
        )

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
        PUBLIC
        "${Boost_LIBRARIES}"
        PUBLIC
        "igl::core"
)

# compile definitions
list(
        APPEND
        CRL_COMPILE_DEFINITIONS #
        PUBLIC
        "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
)

# create target
create_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)
