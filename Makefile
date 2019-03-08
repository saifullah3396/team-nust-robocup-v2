include ${PATH_TO_TEAM_NUST_DIR}/make/common.mk

.PHONY: configure-examples examples clean configure install

configure-examples:
	qibuild configure  ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

examples:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

clean:
	ifdef CROSS
		rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/cross
	endif
	ifdef REMOTE
		rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/remote
	endif

configure:
	qibuild configure ${BUILD_TESTS_FLAG} ${BUILD_EXAMPLES_FLAG} ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX} ${MOTION_PROXY_FLAG} ${VIDEO_PROXY_FLAG}

install:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}
