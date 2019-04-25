ROBOT_VERSION?=V5
BUILD_TYPE?=Release
BUILD_TESTS?=OFF
BUILD_EXAMPLES?=OFF
CMAKE_FLAGS+=-DCMAKE_POSITION_INDEPENDENT_CODE=ON
CMAKE_FLAGS+=-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
CMAKE_FLAGS+=-DBUILD_TNRS_TESTS=$(BUILD_TESTS)
CMAKE_FLAGS+=-DBUILD_TNRS_EXAMPLES=$(BUILD_EXAMPLES)

ifdef CROSS
    CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
    TOOLCHAIN_FLAG=-c $(CROSS)
    CONFFLAGS+=-DTOOLCHAIN_NAME=$(CROSS)
    USE_NAOQI_VIDEO_PROXY?=OFF
else
    ifdef REMOTE
        CONFFLAGS+=-DMODULE_IS_REMOTE=ON
        TOOLCHAIN_FLAG=-c $(REMOTE)
        CONFFLAGS+=-DTOOLCHAIN_NAME=$(REMOTE)
        USE_NAOQI_VIDEO_PROXY?=ON
    else
        ifdef SIMULATION
        CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
        CONFFLAGS+=-DMODULE_IS_LOCAL_SIMULATED=ON
        TOOLCHAIN_FLAG=-c $(SIMULATION)
        CONFFLAGS+=-DTOOLCHAIN_NAME=$(SIMULATION)
        endif
    endif
endif

ifeq ($(USE_NAOQI_MOTION_PROXY), OFF)
	MOTION_PROXY_FLAG=-DUSE_NAOQI_MOTION_PROXY=OFF
    BUILD_PREFIX=$(BUILD_TYPE)
else
	MOTION_PROXY_FLAG=-DUSE_NAOQI_MOTION_PROXY=ON
	BUILD_PREFIX=$(BUILD_TYPE)-Motion
endif

ifeq ($(USE_NAOQI_VIDEO_PROXY), OFF)
	VIDEO_PROXY_FLAG=-DUSE_NAOQI_VIDEO_PROXY=OFF
else
	VIDEO_PROXY_FLAG=-DUSE_NAOQI_VIDEO_PROXY=ON
endif