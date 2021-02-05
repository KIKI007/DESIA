cmake_minimum_required(VERSION 3.1)
project(DESIA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set (CMAKE_CXX_STANDARD 11)

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR      "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL             "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO           "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK             "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE           "Use Embree"         OFF)
option(LIBIGL_WITH_LIM              "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB           "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK            "Use MOSEK"          OFF)
option(LIBIGL_WITH_NANOGUI          "Use Nanogui menu"   ON)
option(LIBIGL_WITH_OPENGL           "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW      "Use GLFW"           ON)
option(LIBIGL_WITH_PNG              "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON           "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN           "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE         "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER           "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML              "Use XML"            OFF)
option(LIBIGL_WITH_SHAPEOP          "Use ShapeOp"        OFF)
option(LIBIGL_WITH_LIBCCD          "Use LibCCD"          OFF)

################################################################################

#add_subdirectory(libigl)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libigl/shared/cmake")
include(libigl)
add_definitions(-DLIBIGL_PATH="${LIBIGL_INCLUDE_DIR}/..")


### add boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.66.0)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

##add Mosek
find_package(MOSEK REQUIRED)

# Add your project file
file(GLOB SRCFILES
        src/*.cpp
        src/*.h
        src/Voxel/*.cpp
        src/Voxel/*.h
        src/ContactGraph/*.h
        src/ContactGraph/*.cpp)
add_executable(${PROJECT_NAME}_bin ${SRCFILES})
target_include_directories(${PROJECT_NAME}_bin PUBLIC SYSTEM ${MOSEK_INCLUDE_DIRS})
target_link_libraries(
        ${PROJECT_NAME}_bin
        igl::core
        igl::viewer ${Boost_LIBRARIES} ${MOSEK_LIBRARIES})