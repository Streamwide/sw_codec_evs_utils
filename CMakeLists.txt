cmake_minimum_required(VERSION 3.16)
project(sw_codec_evs_utils C)

set (DIRLIBEVS              ./c-code)

set (EVS_VERSION h00)

include(FetchContent)
FetchContent_Declare(
  ts26443
  URL      https://www.3gpp.org/ftp/Specs/archive/26_series/26.443/26443-${EVS_VERSION}.zip
)
FetchContent_MakeAvailable(ts26443)

file(ARCHIVE_EXTRACT
    INPUT ${ts26443_SOURCE_DIR}/26443-${EVS_VERSION}-ANSI-C_source_code.zip
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

