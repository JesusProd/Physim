cmake_minimum_required(VERSION 3.6)

set( CMAKE_CXX_STANDARD 14 )
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include (${CMAKE_CURRENT_SOURCE_DIR}/CMakeLocal.txt)

project(PhySim)
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

##########################################################
################## Set PhySim includes ###################
##########################################################

# Self
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/external/tinyobj)

# Required
include_directories(${EIGEN_DIR})
include_directories(${LIBIGL_DIR}/include)
include_directories(${LIBIGL_DIR}/external/tetgen)
include_directories(${LIBIGL_DIR}/external/triangle)

# Optional
include_directories(${KNITROSAMP_INC_DIR})
include_directories(${KNITROCORE_INC_DIR})
include_directories(${SS_SPQR_INC_DIR})
include_directories(${CHOLMOD_INC_DIR})
include_directories(${SSPARSE_INC_DIR})
include_directories(${CUDA_INC_DIR})


##########################################################
################## Setup LibIGL build ####################
##########################################################

option(LIBIGL_USE_STATIC_LIBRARY		"Use LibIGL as static library"		OFF)
option(LIBIGL_WITH_VIEWER      			"Use OpenGL viewer"  				OFF)
option(LIBIGL_WITH_NANOGUI     			"Use Nanogui menu"   				OFF)
option(LIBIGL_WITH_EMBREE      			"Use Embree"         				OFF)
option(LIBIGL_WITH_CGAL             	"Use CGAL"           				OFF)
option(LIBIGL_WITH_CORK             	"Use CORK"           				OFF)
option(LIBIGL_WITH_MATLAB           	"Use Matlab"         				OFF)
option(LIBIGL_WITH_MOSEK            	"Use MOSEK"          				OFF)
option(LIBIGL_WITH_TRIANGLE           	"Use TRIANGLE"          			OFF)

##########################################################
################# Gather source files ####################
##########################################################

file(GLOB SRCFILES 
	source/PhySim/*.cpp
	source/PhySim/Physics/*.cpp
	source/PhySim/Physics/Boundary/*.cpp
	source/PhySim/Physics/Colliders/*.cpp
	source/PhySim/Physics/Elements/*.cpp
	source/PhySim/Physics/Parameters/*.cpp
	source/PhySim/Physics/Simulables/*.cpp
	source/PhySim/Physics/Design/*.cpp
	source/PhySim/Kinematics/*.cpp
	source/PhySim/Geometry/*.cpp
	source/PhySim/Geometry/Polytopes/*.cpp
	source/PhySim/Geometry/Meshes/*.cpp
	source/PhySim/Geometry/Partition/*.cpp
	source/PhySim/Geometry/Samplers/*.cpp
	source/PhySim/Geometry/Volumes/*.cpp
	source/PhySim/Solvers/*.cpp
	source/PhySim/Utils/*.cpp
	source/PhySim/Utils/Auto/*.cpp
	source/PhySim/Utils/Third/*.cpp
	)
	
file(GLOB INCFILES 
	include/PhySim/*.h 
	include/PhySim/Physics/*.h
	include/PhySim/Physics/Boundary/*.h
	include/PhySim/Physics/Colliders/*.h
	include/PhySim/Physics/Elements/*.h
	include/PhySim/Physics/Parameters/*.h
	include/PhySim/Physics/Simulables/*.h
	include/PhySim/Physics/Design/*.h
	include/PhySim/Kinematics/*.h
	include/PhySim/Geometry/*.h
	include/PhySim/Geometry/Polytopes/*.h
	include/PhySim/Geometry/Meshes/*.h
	include/PhySim/Geometry/Partition/*.h
	include/PhySim/Geometry/Samplers/*.h
	include/PhySim/Geometry/Volumes/*.h
	include/PhySim/Solvers/*.h
	include/PhySim/Utils/*.h
	include/PhySim/Utils/Auto/*.h
	include/PhySim/Utils/Third/*.h
	include/PhySim/Utils/Third/*.hpp
	)

source_group(TREE ${PROJECT_SOURCE_DIR}/source/PhySim PREFIX Source FILES ${SRCFILES})
source_group(TREE ${PROJECT_SOURCE_DIR}/include/PhySim PREFIX Source FILES ${INCFILES})

#########################################################
#################### Gather natvis ######################
#########################################################

file(GLOB NVISFILES natvis/*.natvis)

#########################################################
################### Bulding options #####################
#########################################################

# Configure project to build with OpenMP

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -openmp" )

if (WIN32)

# Configure project to build with Bigobj

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -bigobj" )

endif(WIN32)

# Configure project Static/Dynamic linking
if (MSVC)
if (LINKSTATIC)
set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -MTd" )
set( CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -MT" )
else (LINKSTATIC)
set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -MDd" )
set( CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -MD" )
endif (LINKSTATIC)
endif (MSVC)

# User build options

if (WIN32)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LIBIGL_DIR}/external/tetgen/build/Debug/tetgen.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LIBIGL_DIR}/external/triangle/build/Debug/triangle.lib) 

set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LIBIGL_DIR}/external/tetgen/build/Release/tetgen.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LIBIGL_DIR}/external/triangle/build/Release/triangle.lib) 
endif (WIN32)

if (UNIX)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LIBIGL_DIR}/external/tetgen/build/libtetgen.a)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LIBIGL_DIR}/external/triangle/build/libtriangle.a) 

set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LIBIGL_DIR}/external/tetgen/build/libtetgen.a)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LIBIGL_DIR}/external/triangle/build/libtriangle.a) 
endif (UNIX)

########################################################
##################### Link TOBJ ########################
########################################################

set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${PROJECT_SOURCE_DIR}/external/tinyobj/build/Debug/tinyobjloader.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${PROJECT_SOURCE_DIR}/external/tinyobj/build/Release/tinyobjloader.lib)

#########################################################
###################### Link CUDA ########################
#########################################################

if (USE_CUDA)
add_definitions(-DUSE_CUDA)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${CUDA_LIB_DIR}/x64/cuda.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${CUDA_LIB_DIR}/x64/cudart.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${CUDA_LIB_DIR}/x64/cusolver.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${CUDA_LIB_DIR}/x64/cusparse.lib)

set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${CUDA_LIB_DIR}/x64/cuda.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${CUDA_LIB_DIR}/x64/cudart.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${CUDA_LIB_DIR}/x64/cusolver.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${CUDA_LIB_DIR}/x64/cusparse.lib)
endif (USE_CUDA)

#########################################################
##################### Link Knitro #######################
#########################################################

if (USE_KNITRO)
add_definitions(-DUSE_KNITRO)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${KNITRO_LIB_DIR}/knitro1032.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${KNITRO_LIB_DIR}/knitro1032.lib)
endif (USE_KNITRO)

#########################################################
################## Link SuiteSparse #####################
#########################################################

if (USE_SSPARSE)
add_definitions(-DUSE_SSPARSE)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/metis.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libamd.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libcamd.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libspqr.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libcolamd.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libccolamd.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/libcholmod.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${SSPARSE_LIB_DIR}/Debug/suitesparseconfig.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LAPACK_LIB_DIR}/libblas.lib)
set(LIBLINKS_DEBUG_LIST ${LIBLINKS_DEBUG_LIST} ${LAPACK_LIB_DIR}/liblapack.lib)

set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/metis.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libamd.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libcamd.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libspqr.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libcolamd.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libccolamd.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/libcholmod.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${SSPARSE_LIB_DIR}/Release/suitesparseconfig.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LAPACK_LIB_DIR}/libblas.lib)
set(LIBLINKS_RELEASE_LIST ${LIBLINKS_RELEASE_LIST} ${LAPACK_LIB_DIR}/liblapack.lib)
endif (USE_SSPARSE)

#########################################################
#################### Create Library #####################
#########################################################

add_library(${PROJECT_NAME} STATIC ${SRCFILES} ${INCFILES} ${NVISFILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_property(TARGET ${PROJECT_NAME} PROPERTY LIBLINKS_DEBUG ${LIBLINKS_DEBUG_LIST})
set_property(TARGET ${PROJECT_NAME} PROPERTY LIBLINKS_RELEASE ${LIBLINKS_RELEASE_LIST})
