# sw_codec_evs_utils
A wrapper around 3GPP TS 26.442 / TS 26.443. 3GPP interface may be hard to use at first, this is an attempt at simplifying the use of the interface

## Float vs Fixed point
Current code support both code. By default, it is set to use floating point.


## build

This CMakeLists.txt file download EVS fixed point code from 3GPP repository, uncompress it and build it against sw_codec_evs_utils.

    cmake .
    cmake --build . --parallel 5

### build with a particular TS26443 version
By default, it uses version h00 (latest available version as of today - 20220506). It is possible to set anyversion using cmake:

    cmake . -DEVS_VERSION:STRING=g30

### build with FIXED POINT
By default, the lib is built using the FLOATING POINT implementation (TS 26.443), but can be built using the FIXED POINT implementation (TS 26.442):

    cmake . -DWITH_FIXED_POINT=ON

### Logs
No logging facility is provided for now.