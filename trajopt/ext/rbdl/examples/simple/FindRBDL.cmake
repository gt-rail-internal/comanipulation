# Searches for RBDL includes and library files
#
# Sets the variables
#   RBDL_FOUND
#   RBDL_INCLUDE_DIR
#   RBDL_LIBRARIES

SET (RBDL_FOUND FALSE)

FIND_PATH (RBDL_INCLUDE_DIR rbdl.h
	/usr/include
	/usr/include/rbdl
	/usr/local/include
	/usr/local/include/rbdl
	$ENV{HOME}/local/include
	$ENV{HOME}/local/include/rbdl
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_PATH}/include/rbdl
	$ENV{RBDL_INCLUDE_PATH}
	)
FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl	PATHS
	/usr/lib
	/usr/local/lib
	$ENV{HOME}/local/lib
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	)

FIND_LIBRARY (RBDL_LUAMODEL_LIBRARY NAMES rbdl_luamodel	PATHS
	/usr/lib
	/usr/local/lib
	$ENV{HOME}/local/lib
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	)

IF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
	SET (RBDL_FOUND TRUE)
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)

IF (RBDL_LIBRARY AND RBDL_LUAMODEL_LIBRARY)
	SET (RBDL_LIBRARIES ${RBDL_LIBRARY} ${RBDL_LUAMODEL_LIBRARY})
ELSE (RBDL_LIBRARY AND RBDL_LUAMODEL_LIBRARY)
	SET (RBDL_LIBRARIES ${RBDL_LIBRARY})
ENDIF(RBDL_LIBRARY AND RBDL_LUAMODEL_LIBRARY)

IF (RBDL_FOUND)
   IF (NOT RBDL_FIND_QUIETLY)
      MESSAGE(STATUS "Found RBDL: ${RBDL_LIBRARY}")
   ENDIF (NOT RBDL_FIND_QUIETLY)
ELSE (RBDL_FOUND)
   IF (RBDL_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find RBDL")
   ENDIF (RBDL_FIND_REQUIRED)
ENDIF (RBDL_FOUND)

MARK_AS_ADVANCED (
	RBDL_INCLUDE_DIR
	RBDL_LIBRARIES
	)
