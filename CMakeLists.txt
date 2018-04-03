project(Primo CXX)

cmake_minimum_required(VERSION 2.8)

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(EIGEN_INCLUDE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

# additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# setup GLUT
find_package(GLUT)
if(NOT GLUT_FOUNT)
	set(GLUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/freeglut)
endif()
include_directories(${GLUT_INCLUDE_DIR})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found")
endif()

# setup OpenGL
find_package(OpenGL)
include_directories(${OpenGL_INCLUDE_DIRS})
if(NOT OPENGL_FOUND)
    message(ERROR " OpenGL not found")
endif()

# setup OpenMesh
find_package(OpenMesh)
include_directories(${OPENMESH_INCLUDE_DIRS})
if(NOT OPENMESH_FOUND)
    message(ERROR " OpenMesh not found")
endif()
set_property(
    DIRECTORY
    APPEND PROPERTY COMPILE_DEFINITIONS _USE_MATH_DEFINES
)

# collect sources
file(GLOB viewer_sources viewer/*.cc viewer/*.cpp)
file(GLOB viewer_headers viewer/*.hh viewer/*.h)

file(GLOB reconstruct_sources reconstruct/*.cc)
file(GLOB reconstruct_headers reconstruct/*.hh)

file(GLOB ann_sources ANN/*.cpp)
file(GLOB ann_headers ANN/*.h)

file(GLOB primo_headers primo/*.h)

file(GLOB_RECURSE isoex_sources isoex/*.cc)
file(GLOB_RECURSE isoex_headers isoex/*.hh)

# setup Eigen
file(GLOB_RECURSE eigen_sources Eigen/src/*.cpp)
file(GLOB_RECURSE eigen_headers Eigen/src/*.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRECTORY})
include_directories(gmm)

# for isoex
add_definitions(-DINCLUDE_TEMPLATES)

# compile and link
add_library(ann ${ann_sources} ${ann_headers})
add_library(isoex ${isoex_sources} ${isoex_headers})
# add_library(eigen ${eigen_sources} ${eigen_headers})
add_executable(reconstruct ${reconstruct_sources} ${reconstruct_headers})
add_executable(viewer ${viewer_sources} ${viewer_headers})
target_link_libraries(viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES})
target_link_libraries(reconstruct ${OPENMESH_LIBRARIES} isoex)
