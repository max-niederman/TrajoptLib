# Disable in-source builds to prevent source tree corruption
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

cmake_minimum_required(VERSION 3.24)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules"
)

project(TrajoptLib LANGUAGES CXX)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# When building, don't use the install RPATH already (but later on when
# installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to be used when installing, but only if it's not a system directory
list(
    FIND
    CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
    "${CMAKE_INSTALL_PREFIX}/lib"
    isSystemDir
)
if("${isSystemDir}" STREQUAL "-1")
    list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

# Set default build type to release with debug info (i.e. release mode
# optimizations are performed, but debug info still exists).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()

# Generate compile_commands.json by default
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

# Control where the static and shared libraries are built so that on Windows,
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS FALSE)

include(TrajoptLibCompilerFlags)

file(GLOB_RECURSE src src/*.cpp)
add_library(TrajoptLib ${src})
trajoptlib_compiler_flags(TrajoptLib)
target_include_directories(TrajoptLib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

set_target_properties(TrajoptLib PROPERTIES DEBUG_POSTFIX "d")

set_property(TARGET TrajoptLib PROPERTY FOLDER "libraries")
target_compile_definitions(TrajoptLib PRIVATE TRAJOPT_EXPORTS)

include(FetchContent)

# fmt dependency
set(BUILD_SHARED_LIBS_SAVE ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS_SAVE})
set(FMT_INSTALL ON)
fetchcontent_declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 10.1.1
)
fetchcontent_makeavailable(fmt)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVE})

if(
    ${OPTIMIZER_BACKEND} STREQUAL "casadi"
    AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux"
    AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
)
    target_compile_definitions(fmt PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
endif()

target_link_libraries(TrajoptLib PUBLIC fmt)

# json dependency
set(BUILD_SHARED_LIBS_SAVE ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
set(JSON_Install ON)
fetchcontent_declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json
    GIT_TAG v3.11.2
)
fetchcontent_makeavailable(nlohmann_json)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVE})

target_link_libraries(TrajoptLib PUBLIC nlohmann_json)

set(OPTIMIZER_BACKEND "casadi" CACHE STRING "Optimizer backend")
set_property(CACHE OPTIMIZER_BACKEND PROPERTY STRINGS casadi sleipnir)

if(${OPTIMIZER_BACKEND} STREQUAL "casadi")
    message(STATUS "Using CasADi optimizer")
    include(FetchCasADi)
    fetch_casadi()
    target_compile_definitions(TrajoptLib PRIVATE OPTIMIZER_BACKEND_CASADI)
    target_include_directories(TrajoptLib SYSTEM PRIVATE ${CASADI_INCLUDEDIR})
    target_link_directories(TrajoptLib PRIVATE ${CASADI_LIBDIR})
    target_link_libraries(TrajoptLib PRIVATE casadi)

    # Add current directory to RPATH
    if(APPLE)
        set_property(
            TARGET TrajoptLib
            APPEND
            PROPERTY INSTALL_RPATH "@loader_path/../lib;@loader_path"
        )
    elseif(UNIX)
        set_property(
            TARGET TrajoptLib
            APPEND
            PROPERTY INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN"
        )
    endif()

    # Install CasADi libraries since FetchContent setting that up properly
    install(FILES ${CASADI_INSTALL_LIBS} DESTINATION ${CASADI_INSTALL_DEST})
elseif(${OPTIMIZER_BACKEND} STREQUAL "sleipnir")
    message(STATUS "Using Sleipnir optimizer")
    include(FetchSleipnir)
    fetch_sleipnir()
    target_compile_definitions(TrajoptLib PRIVATE OPTIMIZER_BACKEND_SLEIPNIR)
    target_link_libraries(TrajoptLib PRIVATE Sleipnir)

    # Install Sleipnir manually because EXCLUDE_FROM_ALL was used during import
    install(TARGETS Sleipnir)
endif()

target_include_directories(
    TrajoptLib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
    TARGETS TrajoptLib
    EXPORT TrajoptLibTargets
    LIBRARY
    DESTINATION lib
    ARCHIVE
    DESTINATION lib
    RUNTIME
    DESTINATION bin
    INCLUDES DESTINATION include
)
export(TARGETS TrajoptLib FILE TrajoptLib.cmake NAMESPACE TrajoptLib::)
install(DIRECTORY include/ DESTINATION "include")
install(
    EXPORT TrajoptLibTargets
    FILE TrajoptLib.cmake
    NAMESPACE TrajoptLib::
    DESTINATION lib/cmake/TrajoptLib
)

include(CMakePackageConfigHelpers)

# Generate the config file that includes the exports
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/TrajoptLibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
    INSTALL_DESTINATION "lib/cmake/TrajoptLib"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Install the config file
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
    DESTINATION lib/cmake/TrajoptLib
)

# GoogleTest dependency (static linkage)
set(BUILD_SHARED_LIBS_SAVE ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
option(INSTALL_GMOCK "Install GoogleTest's GMock" OFF)
option(INSTALL_GTEST "Install GoogleTest's GTest" OFF)
fetchcontent_declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    CMAKE_ARGS
    "-DCMAKE_BUILD_TYPE=ReleaseWithDebInfo"
    FIND_PACKAGE_ARGS NAMES GTest
)
fetchcontent_makeavailable(googletest)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVE})

include(CMakeDependentOption)

cmake_dependent_option(
    BUILD_TESTING
    "Enable creation of tests."
    OFF
    "PROJECT_IS_TOP_LEVEL"
    OFF
)
if(BUILD_TESTING)
    enable_testing()
    include(GoogleTest)
endif()

# Build TrajoptLib tests
if(BUILD_TESTING)
    file(GLOB_RECURSE TrajoptLib_test_src test/src/*.cpp)
    add_executable(TrajoptLibTest ${TrajoptLib_test_src})
    trajoptlib_compiler_flags(TrajoptLibTest)
    target_include_directories(
        TrajoptLibTest
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/test/include
            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/units/include
    )
    target_link_libraries(
        TrajoptLibTest
        PRIVATE TrajoptLib fmt::fmt GTest::gtest GTest::gtest_main
    )
    if(NOT CMAKE_TOOLCHAIN_FILE)
        gtest_discover_tests(TrajoptLibTest)
    endif()
endif()

# Build examples and example tests
include(TrajoptLibSubdirList)
trajoptlib_subdir_list(EXAMPLES ${CMAKE_CURRENT_SOURCE_DIR}/examples)
foreach(example ${EXAMPLES})
    # Build example
    file(GLOB_RECURSE sources examples/${example}/src/*.cpp)
    add_executable(${example} ${sources})
    trajoptlib_compiler_flags(${example})
    target_include_directories(
        ${example}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
    )
    target_link_libraries(${example} PRIVATE TrajoptLib)

    # Build example test if files exist for it
    if(
        BUILD_TESTING
        AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test
    )
        file(GLOB_RECURSE test_sources examples/${example}/test/*.cpp)
        add_executable(${example}Test ${sources} ${test_sources})
        target_include_directories(
            ${example}Test
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/src
                ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test
        )
        trajoptlib_compiler_flags(${example}Test)
        target_compile_definitions(${example}Test PUBLIC RUNNING_TESTS)
        target_include_directories(
            ${example}Test
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
        )
        target_link_libraries(
            ${example}Test
            PRIVATE TrajoptLib GTest::gtest GTest::gtest_main
        )
        if(NOT CMAKE_TOOLCHAIN_FILE)
            gtest_discover_tests(${example}Test)
        endif()
    endif()
endforeach()

# Build the Java bindings

option(WITH_JAVA "Build Java bindings" OFF)
cmake_dependent_option(
    JAVA_INSTALL_JAR
    "Install jar file."
    ON
    "WITH_JAVA"
    ON
)

if(WITH_JAVA)
    add_library(
        TrajoptLib-java
        SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/jni/TrajoptLibJNI.cpp
    )
    target_link_libraries(TrajoptLib-java PRIVATE TrajoptLib)
    target_include_directories(
        TrajoptLib-java
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/jni.h
    )
    if(
        ${CMAKE_SYSTEM_NAME} MATCHES "MINGW"
        OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS"
        OR WIN32
    )
        target_include_directories(
            TrajoptLib-java
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/win32
        )
    elseif(APPLE)
        target_include_directories(
            TrajoptLib-java
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/darwin
        )
    elseif(UNIX)
        target_include_directories(
            TrajoptLib-java
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/linux
        )
    endif()
    include(InstallJavaLibs)
    installjavalibs()

    # Build Jar containing embedded native libraries
    if(JAVA_INSTALL_JAR)
        if(UNIX)
            install(
                CODE
                    "execute_process(COMMAND ./gradlew jar WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/java)"
            )
        elseif(
            ${CMAKE_SYSTEM_NAME} MATCHES "MINGW"
            OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS"
            OR WIN32
        )
            install(
                CODE
                    "execute_process(COMMAND .\\gradlew.bat jar WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/java)"
            )
        endif()
        file(GLOB jar_files "${CMAKE_CURRENT_SOURCE_DIR}/java/build/libs/*jar")
        install(FILES ${jar_files} DESTINATION java)
    endif()
endif()
