macro(trajoptlib_compiler_flags target)
    if(NOT MSVC)
        target_compile_options(
            ${target}
            PRIVATE
                -Wall
                -pedantic
                -Wextra
                -Werror
                -Wno-unused-parameter
                -Wno-missing-braces
        )

        # clang 18 warns on `operator"" _a` in dependencies
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "18"
        )
            target_compile_options(
                ${target}
                PRIVATE -Wno-deprecated-literal-operator
            )
        endif()

        if(
            ${OPTIMIZER_BACKEND} STREQUAL "casadi"
            AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux"
            AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
        )
            target_compile_definitions(
                ${target}
                PRIVATE _GLIBCXX_USE_CXX11_ABI=0
            )
        endif()

        # Disable warning false positives in Eigen
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "8"
        )
            target_compile_options(${target} PRIVATE -Wno-class-memaccess)
        endif()
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "11"
        )
            target_compile_options(${target} PRIVATE -Wno-maybe-uninitialized)
        endif()
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "12"
        )
            target_compile_options(${target} PRIVATE -Wno-array-bounds)
        endif()

        # Disable deprecated-anon-enum-enum-conversion warning in Eigen
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "13"
        )
            target_compile_options(
                ${target}
                PRIVATE -Wno-deprecated-anon-enum-enum-conversion
            )
        endif()

        # Disable warning false positives in fmt
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "13"
        )
            target_compile_options(
                ${target}
                PRIVATE -Wno-dangling-reference -Wno-stringop-overflow
            )
        endif()
    else()
        target_compile_options(
            ${target}
            PRIVATE /wd4146 /wd4244 /wd4251 /wd4267 /WX
        )
    endif()

    target_compile_features(${target} PUBLIC cxx_std_20)
    if(MSVC)
        target_compile_options(${target} PUBLIC /bigobj)
    endif()
endmacro()
