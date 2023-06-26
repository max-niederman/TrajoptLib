macro(fetch_sleipnir)
  include(FetchContent)

  FetchContent_Declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
    GIT_TAG f497fc32e0488e16c567bdcbb6e9a1ca95befabc
  )

  FetchContent_GetProperties(Sleipnir)
  if(NOT Sleipnir_POPULATED)
    FetchContent_Populate(Sleipnir)
    add_subdirectory(${sleipnir_SOURCE_DIR} ${sleipnir_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
endmacro()
