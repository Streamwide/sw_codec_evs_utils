cmake_minimum_required(VERSION 3.16)
project(sw_codec_evs_utils C)

set (DIRLIBEVS              ./c-code)

set(EVS_VERSION "h00" CACHE STRING "The TS 26.442/26.443 version to use")
set(EVS_NUMBER "443")

option(WITH_FIXED_POINT "Use FIXED POINT implementation instead of FLOATING POINT" OFF )

if (WITH_FIXED_POINT)
    add_definitions(-DEVS_FX=1)
    set(EVS_NUMBER "442")
endif (WITH_FIXED_POINT)

set(EVS_URL https://www.3gpp.org/ftp/Specs/archive/26_series/26.${EVS_NUMBER}/26${EVS_NUMBER}-${EVS_VERSION}.zip)


message("Will build using ${EVS_URL}")

include(FetchContent)
FetchContent_Declare(
  ts2644x
  URL ${EVS_URL}
)
FetchContent_MakeAvailable(ts2644x)

file(ARCHIVE_EXTRACT
    INPUT ${ts2644x_SOURCE_DIR}/26${EVS_NUMBER}-${EVS_VERSION}-ANSI-C_source_code.zip
    VERBOSE
)


add_compile_options(
    -Wcast-qual -Wall -W -Wextra -Wno-long-long
    -Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes
    -Werror-implicit-function-declaration -std=c11
)
#add_compile_options(-g)
add_compile_options(-O2)

add_definitions(-DRELEASE=1)
#add_definitions(-DNDEBUG -fPIC)


set(DIRS "basic_math;basic_op;lib_com;lib_dec;lib_enc")
file(GLOB source_files "")
FOREACH(DIR ${DIRS})
    MESSAGE("<<${DIR}>>")
    include_directories(
        ${DIRLIBEVS}/${DIR}
    )
    file (GLOB tmp ${DIRLIBEVS}/${DIR}/*.c)
    list (APPEND source_files ${tmp})
ENDFOREACH()

include_directories(include)
list (APPEND source_files src/sw_codec_evs_utils.c)


#Generate the static library from the sources
add_library(sw_codec_evs_utils STATIC ${source_files})

