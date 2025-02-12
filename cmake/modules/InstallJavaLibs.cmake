macro(installjavalibs)
    set(JAVA_LIB_TARGET_PATH
        ${CMAKE_CURRENT_SOURCE_DIR}/java/src/main/resources
    )
    if(
        ${CMAKE_SYSTEM_NAME} MATCHES "MINGW"
        OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS"
        OR WIN32
    )
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(JAVA_LIB_TARGET_PATH ${JAVA_LIB_TARGET_PATH}/windows/x86_64)
        elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
            set(JAVA_LIB_TARGET_PATH ${JAVA_LIB_TARGET_PATH}/windows/x86)
        endif()
    elseif(APPLE)
        if(CMAKE_APPLE_SILICON_PROCESSOR MATCHES "arm64")
            set(JAVA_LIB_TARGET_PATH ${JAVA_LIB_TARGET_PATH}/osx/arm64)
        elseif(CMAKE_APPLE_SILICON_PROCESSOR MATCHES "x86_64")
            set(JAVA_LIB_TARGET_PATH ${JAVA_LIB_TARGET_PATH}/osx/x86_64)
        endif()
    elseif(UNIX)
        set(JAVA_LIB_TARGET_PATH ${JAVA_LIB_TARGET_PATH}/linux/x86_64)
    endif()

    message(STATUS "Path to use for java: ${JAVA_LIB_TARGET_PATH}")
    install(
        TARGETS TrajoptLib
        LIBRARY
        DESTINATION ${JAVA_LIB_TARGET_PATH}
        RUNTIME
        DESTINATION ${JAVA_LIB_TARGET_PATH}
    )
    install(
        TARGETS TrajoptLib-java
        LIBRARY
        DESTINATION ${JAVA_LIB_TARGET_PATH}
        RUNTIME
        DESTINATION ${JAVA_LIB_TARGET_PATH}
    )

    if(OPTIMIZER_BACKEND STREQUAL "casadi")
        install(
            FILES ${CASADI_INSTALL_LIBS}
            DESTINATION ${JAVA_LIB_TARGET_PATH}
        )
    endif()

    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/GenerateJNILibsJSON.cmake.in"
        "${CMAKE_BINARY_DIR}/GenerateJNILibsJSON.cmake"
        @ONLY
    )
    install(SCRIPT "${CMAKE_BINARY_DIR}/GenerateJNILibsJSON.cmake")
endmacro()
