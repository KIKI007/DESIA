cmake_minimum_required(VERSION 3.5)

project(plotApp)

file(GLOB CRL_SOURCES #
        "main.cpp" #
        )

list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "igl::core"
        "igl::glfw"
        "igl::imgui"
        "voxel")

list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )

list(APPEND CRL_TARGET_LINK_LIBS #
        PUBLIC
        "igl::core"
        PUBLIC
        "igl::glfw"
        PUBLIC
        "voxel"
        PUBLIC
        "igl::imgui"
        )

list(APPEND CRL_COMPILE_DEFINITIONS #
        PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}"#
        )

create_app(
        ${PROJECT_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)
