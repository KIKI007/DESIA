# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------

# download project and add it to build tree
macro(fetch_and_add what)
    FetchContent_GetProperties("${what}")
    if (NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
        add_subdirectory(${${what}_SOURCE_DIR} ${what})
    endif ()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

# only download project
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if (NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif ()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()


# -----------------------------------------------------------------------------
# dependencies
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Eigen
# download as zip, so it's a bit smaller than downloading the whole repo
if (NOT TARGET eigen)
    FetchContent_Declare(
            eigen #
            URL https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip #
            URL_HASH MD5=888aab45512cc0c734b3e8f60280daba #
    )
    fetch(eigen)
    add_library(eigen INTERFACE)
    add_library(Eigen3::Eigen ALIAS eigen)
    target_include_directories(eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif ()

# -----------------------------------------------------------------------------
# json
FetchContent_Declare(
        json #
        GIT_REPOSITORY https://github.com/nlohmann/json.git #
        GIT_TAG 700b95f4473cac1d900b807541daad0ca3d98f52 #
)
fetch_and_add(json)

# -----------------------------------------------------------------------------
# json
FetchContent_Declare(
        libigl #
        GIT_REPOSITORY https://github.com/libigl/libigl.git #
        GIT_TAG 238a607 #
)
option(LIBIGL_IMGUI  "Build target igl::imgui"     ON)
option(LIBIGL_GLFW   "Build target igl::glfw" ON)
fetch_and_add(libigl)