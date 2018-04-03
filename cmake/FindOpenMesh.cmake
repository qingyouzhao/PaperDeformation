#
# Try to find OPENMESH
# Once done this will define
#  
# OPENMESH_FOUND           - system has OPENMESH
# OPENMESH_INCLUDE_DIRS    - the OPENMESH include directories
# OPENMESH_LIBRARIES       - Link these to use OPENMESH
#
# Copyright 2013 Computer Graphics Group, RWTH Aachen University
# Authors: Jan Möbius <moebius@cs.rwth-aachen.de>
#          Hans-Christian Ebke <ebke@cs.rwth-aachen.de>
#
# This file is part of OpenMesh.
#===========================================================================
#                                                                           
#                               OpenMesh                                    
#      Copyright (C) 2001-2012 by Computer Graphics Group, RWTH Aachen      
#                           www.openmesh.org                                
#                                                                           
#---------------------------------------------------------------------------
#  This file is part of OpenMesh.                                           
#                                                                           
#  OpenMesh is free software: you can redistribute it and/or modify         
#  it under the terms of the GNU Lesser General Public License as           
#  published by the Free Software Foundation, either version 3 of           
#  the License, or (at your option) any later version with the              
#  following exceptions:                                                    
#                                                                           
#  If other files instantiate templates or use macros                       
#  or inline functions from this file, or you compile this file and         
#  link it with other files to produce an executable, this file does        
#  not by itself cause the resulting executable to be covered by the        
#  GNU Lesser General Public License. This exception does not however       
#  invalidate any other reasons why the executable file might be            
#  covered by the GNU Lesser General Public License.                        
#                                                                           
#  OpenMesh is distributed in the hope that it will be useful,              
#  but WITHOUT ANY WARRANTY; without even the implied warranty of           
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            
#  GNU Lesser General Public License for more details.                      
#                                                                           
#  You should have received a copy of the GNU LesserGeneral Public          
#  License along with OpenMesh.  If not,                                    
#  see <http://www.gnu.org/licenses/>.                                      
#                                                                           
#===========================================================================

IF (OPENMESH_INCLUDE_DIR)
  # Already in cache, be silent
  SET(OPENMESH_FIND_QUIETLY TRUE)
ENDIF (OPENMESH_INCLUDE_DIR)

FIND_PATH(OPENMESH_INCLUDE_DIR OpenMesh/Core/Mesh/PolyMeshT.hh
	  PATHS /usr/local/include 
                /usr/include 
                /usr/local/OpenMesh-2.0rc4/include
                "${CMAKE_SOURCE_DIR}/OpenMesh/src"
                "${CMAKE_SOURCE_DIR}/libs_required/OpenMesh/src"
                "${CMAKE_SOURCE_DIR}/../OpenMesh/src"
                "${CMAKE_SOURCE_DIR}/OpenMesh"
                /ACG/acgdev/gcc-4.0-x86_64/OM2/OpenMesh-2.0/installed/include
		    "C:\\Program Files\\OpenMesh 2.0\\include"
                )
SET(OPENMESH_LIBRARY_DIR NOTFOUND CACHE PATH "The directory where the OpenMesh libraries can be found.")
SET(SEARCH_PATHS
    "${OPENMESH_INCLUDE_DIR}/../lib"
    "${OPENMESH_INCLUDE_DIR}/../lib/OpenMesh"
    "${OPENMESH_LIBRARY_DIR}"
    "${OPENMESH_LIBRARY_DIR}/OpenMesh/lib")
FIND_LIBRARY(OPENMESH_CORE_LIBRARY NAMES OpenMeshCored OpenMeshCore PATHS ${SEARCH_PATHS})
FIND_LIBRARY(OPENMESH_TOOLS_LIBRARY NAMES OpenMeshToolsd OpenMeshTools PATHS ${SEARCH_PATHS})

INCLUDE (FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenMesh DEFAULT_MSG OPENMESH_CORE_LIBRARY OPENMESH_TOOLS_LIBRARY OPENMESH_INCLUDE_DIR)

IF (OPENMESH_FOUND)
   SET(OPENMESH_LIBRARIES "${OPENMESH_CORE_LIBRARY}" "${OPENMESH_TOOLS_LIBRARY}")
   SET(OPENMESH_INCLUDE_DIRS "${OPENMESH_INCLUDE_DIR}")
ENDIF (OPENMESH_FOUND)
