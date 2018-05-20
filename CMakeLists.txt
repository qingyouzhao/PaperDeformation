project(Primo CXX)

cmake_minimum_required(VERSION 2.8)



# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(EIGEN_INCLUDE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)
set(GLM_INCLUDE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/glm)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
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
file(GLOB primo_viewer_sources primo_viewer/*.cc primo_viewer/*.cpp)
file(GLOB primo_viewer_headers primo_viewer/*.hh primo_viewer/*.h)

file(GLOB paper_viewer_sources paper_viewer/*.cc paper_viewer/*.cpp)
file(GLOB paper_viewer_headers paper_viewer/*.hh paper_viewer/*.h)

#file(GLOB reconstruct_sources reconstruct/*.cc)
#file(GLOB reconstruct_headers reconstruct/*.hh)

#file(GLOB ann_sources ANN/*.cpp)
#file(GLOB ann_headers ANN/*.h)

#file(GLOB primo_headers primo/*.h)

#file(GLOB_RECURSE isoex_sources isoex/*.cc)
#file(GLOB_RECURSE isoex_headers isoex/*.hh)

# setup Eigen
# file(GLOB_RECURSE eigen_sources Eigen/src/*.cpp)
# file(GLOB_RECURSE eigen_headers Eigen/src/*.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRECTORY})
include_directories(${GLM_INCLUDE_DIRECTORY})
# no need for gmm here
# include_directories(gmm)

# for isoex
add_definitions(-DINCLUDE_TEMPLATES)

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          OFF)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  OFF)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)

# compile and link
# add_library(ann ${ann_sources} ${ann_headers})
# no need for isoex, reconstruct here.
# add_library(isoex ${isoex_sources} ${isoex_headers})
# add_library(eigen ${eigen_sources} ${eigen_headers})
# add_executable(reconstruct ${reconstruct_sources} ${reconstruct_headers})
add_executable(primo_viewer ${primo_viewer_sources} ${primo_viewer_headers})
add_executable(paper_viewer ${paper_viewer_sources} ${paper_viewer_headers} )

if(WIN32)
target_link_libraries(primo_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} )
else()
target_link_libraries(primo_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} -lpthread)
endif()

if(WIN32)
target_link_libraries(paper_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} igl::core igl::triangle )
else()
target_link_libraries(paper_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} igl::core igl::triangle -lpthread)
endif()