include ${PATH_TO_TEAM_NUST_DIR}/make/common.mk

.PHONY: tests clean qi

configure-tests:
	qibuild configure -DBUILD_TNRS_TESTS=ON ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

tests:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

clean:
ifdef CROSS
	rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/build-${CROSS}/tnrs-behavior-manager
endif
ifdef REMOTE
	rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/build-${REMOTE}/tnrs-behavior-manager
endif
	
configure:
	qibuild configure -DBUILD_TNRS_TESTS=OFF ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

install:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}
	
