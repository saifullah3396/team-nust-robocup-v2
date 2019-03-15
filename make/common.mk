ROBOT_VERSION=V50

ifdef DEBUG
    CONFFLAGS=-DCMAKE_BUILD_TYPE=Debug
else
    CONFFLAGS=-DCMAKE_BUILD_TYPE=Release
endif

ifdef CROSS
    CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
    TOOLCHAIN_FLAG=-c $(CROSS)
    CONFFLAGS+=-DTOOLCHAIN_NAME=$(CROSS)
    CONFFLAGS+=-DUSE_NAOQI_VIDEO_PROXY=OFF
else
    ifdef REMOTE
        CONFFLAGS+=-DMODULE_IS_REMOTE=ON
        TOOLCHAIN_FLAG=-c $(REMOTE)
        CONFFLAGS+=-DTOOLCHAIN_NAME=$(REMOTE)
    else
        ifdef SIMULATION
        CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
        CONFFLAGS+=-DMODULE_IS_LOCAL_SIMULATED=ON
        TOOLCHAIN_FLAG=-c $(SIMULATION)
        CONFFLAGS+=-DTOOLCHAIN_NAME=$(SIMULATION)
    else
        $(error Please define either CROSS=<cross toolchain name>, REMOTE=<remote toolchain name> or SIMULATION=<sim toolchain name> to continue)
    endif
    endif
endif

ifdef BUILD_TESTS
  BUILD_TESTS_FLAG=-DBUILD_TNRS_TESTS=$(BUILD_TESTS)
endif

ifdef BUILD_EXAMPLES
  BUILD_EXAMPLES_FLAG=-DBUILD_TNRS_EXAMPLES=$(BUILD_EXAMPLES)
endif

ifeq ($(USE_NAOQI_MOTION_PROXY), OFF)
	MOTION_PROXY_FLAG=-DUSE_NAOQI_MOTION_PROXY=OFF
	ifdef DEBUG
		BUILD_PREFIX=Debug
	else
		BUILD_PREFIX=Release
	endif
else
	MOTION_PROXY_FLAG=-DUSE_NAOQI_MOTION_PROXY=ON
	ifdef DEBUG
		BUILD_PREFIX=Debug-Motion
	else
		BUILD_PREFIX=Release-Motion
	endif
endif

ifeq ($(USE_NAOQI_VIDEO_PROXY), OFF)
	VIDEO_PROXY_FLAG=-DUSE_NAOQI_VIDEO_PROXY=OFF
else
	VIDEO_PROXY_FLAG=-DUSE_NAOQI_VIDEO_PROXY=ON
endif

CONFFLAGS+=-DROBOT_VERSION=$(ROBOT_VERSION)
