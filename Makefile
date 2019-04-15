include $(PATH_TO_TEAM_NUST_DIR)/make/common.mk
.PHONY: clean configure install

interface:
	catkin_make -C $(PATH_TO_TEAM_NUST_DIR)/resources/team_nust_nao_interface_ws -j4

configure:
	@if [ "$(strip $(CONFFLAGS))" = "" ]; then\
		echo "Please define either CROSS=<cross toolchain name>, REMOTE=<remote toolchain name> or SIMULATION=<sim toolchain name> to continue)";\
		exit 1; \
	fi
	qibuild configure $(CMAKE_FLAGS) $(CONFFLAGS) $(TOOLCHAIN_FLAG) --build-prefix $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX) $(MOTION_PROXY_FLAG) $(VIDEO_PROXY_FLAG) -DROBOT_VERSION=$(ROBOT_VERSION)

install:
	@if [ "$(strip $(CONFFLAGS))" = "" ]; then\
		echo "Please define either CROSS=<cross toolchain name>, REMOTE=<remote toolchain name> or SIMULATION=<sim toolchain name> to continue)";\
		exit 1; \
	fi
	qibuild make $(TOOLCHAIN_FLAG) --build-prefix $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX)

clean:
	ifdef CROSS
		rm -rf $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX)/cross
		rm -rf $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX)/build-cross
	endif
	ifdef REMOTE
		rm -rf $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX)/remote
		rm -rf $(PATH_TO_TEAM_NUST_DIR)/build-$(ROBOT_VERSION)/$(BUILD_PREFIX)/build-remote
	endif

all: configure install interface