set(RCL_VERSION 1.13.0)

IF(CUSTOM_RCL_PATH)
    FIND_PATH (RCL_INCLUDE_DIR rcl/Api.h
        PATHS
        ${CUSTOM_RCL_PATH}/include
        NO_DEFAULT_PATH
    )
    file(GLOB RCL_LIBRARY
        "${CUSTOM_RCL_PATH}/lib/*.so"
        "${CUSTOM_RCL_PATH}/lib/*.so.*"
        "${CUSTOM_RCL_PATH}/lib/*.a"
    )
ELSE(CUSTOM_RCL_PATH)
    MESSAGE (SEND_ERROR " Could not find RCL.")
ENDIF(CUSTOM_RCL_PATH)

IF (RCL_INCLUDE_DIR AND RCL_LIBRARY)
    SET (RCL_FOUND TRUE)
ELSE(RCL_INCLUDE_DIR AND RCL_LIBRARY)
    IF(RCL_FIND_REQUIRED)
        MESSAGE (SEND_ERROR " Could not find RCL.")
        MESSAGE (SEND_ERROR " Try setting CUSTOM_RCL_PATH in FindRCL.cmake force CMake to use the desired directory.")
    ELSE(RCL_FIND_REQUIRED)
        MESSAGE (STATUS " Could not find RCL.")
        MESSAGE (STATUS " Try setting CUSTOM_RCL_PATH in FindRCL.cmake force CMake to use the desired directory.")
    ENDIF(RCL_FIND_REQUIRED)
ENDIF (RCL_INCLUDE_DIR AND RCL_LIBRARY)

IF (RCL_FOUND)
    IF (NOT RCL_FIND_QUIETLY)
        MESSAGE(STATUS "Found RCL: ${RCL_LIBRARY}")
    ENDIF (NOT RCL_FIND_QUIETLY)
    foreach ( COMPONENT ${RCL_FIND_COMPONENTS} )
        IF (RCL_${COMPONENT}_FOUND)
            IF (NOT RCL_FIND_QUIETLY)
                MESSAGE(STATUS "Found RCL ${COMPONENT}: ${RCL_${COMPONENT}_LIBRARY}")
            ENDIF (NOT RCL_FIND_QUIETLY)
        ELSE (RCL_${COMPONENT}_FOUND)
            MESSAGE(ERROR " Could not find RCL ${COMPONENT}")
        ENDIF (RCL_${COMPONENT}_FOUND)
    endforeach ( COMPONENT )
ENDIF (RCL_FOUND)

MARK_AS_ADVANCED (
    RCL_INCLUDE_DIR
    RCL_LIBRARY
)
