# mnu_module.mk - build rules for a MiSTer menu module (.mnu)
#
# A core keeps its menu module with its own sources, in <core>/menu/:
#
#   <core>/menu/Makefile:
#       MNU_NAME     = Maximig                            # -> menu/Maximig.mnu
#       MAIN_MISTER ?= $(abspath ../../MiSTer_Main/Main_MiSTer)
#       MNU_SD_DIR  ?= /media/fat                         # where the rbf lives on the card
#       include $(MAIN_MISTER)/mnu_module.mk
#
#   make            build $(MNU_NAME).mnu from the *.cpp and *.c in this folder
#   make deploy     copy it to root@$(MNU_HOST):$(MNU_SD_DIR)/$(MNU_SD_NAME)
#   make reload     ask the running MiSTer to reload its module (mnu_reload)
#   make clean
#
# Knobs: MNU_SRC (sources), MNU_INCLUDES (extra -I flags), MNU_CFLAGS_EXTRA,
# MNU_SD_NAME (name on the card, default $(MNU_NAME).mnu; a dated rbf such as
# Name_20261029.rbf also picks up Name.mnu), MNU_HOST (default mister), BASE
# (toolchain prefix, the same one that builds MiSTer).
#
# A module talks to MiSTer only through the mnu_api.h vtable (and a family
# header such as mnu_minimig.h), so it links with --no-undefined to prove it;
# -fno-gnu-unique keeps dlclose() able to really unload it.

BASE        ?= arm-none-linux-gnueabihf
CC           = $(BASE)-gcc
READELF      = $(BASE)-readelf
MNU_HOST    ?= mister
MNU_SD_NAME ?= $(strip $(MNU_NAME)).mnu
MNU_SRC     ?= $(wildcard *.cpp) $(wildcard *.c)
MNU_BUILD   ?= build
MNU_OUT      = $(strip $(MNU_NAME)).mnu

ifeq ($(strip $(MNU_NAME)),)
$(error set MNU_NAME before including mnu_module.mk)
endif
# make keeps the padding before a trailing comment in a value; never let it reach a path
override MNU_NAME    := $(strip $(MNU_NAME))
override MNU_SD_DIR  := $(strip $(MNU_SD_DIR))
override MNU_SD_NAME := $(strip $(MNU_SD_NAME))
override MNU_HOST    := $(strip $(MNU_HOST))
ifeq ($(wildcard $(MAIN_MISTER)/mnu_api.h),)
$(error MAIN_MISTER=$(MAIN_MISTER) does not contain mnu_api.h)
endif

# Main's own headers (reached from mnu_minimig.h or support/*) need Main's library paths.
MNU_HOST_INCLUDES = -I$(MAIN_MISTER) $(addprefix -I$(MAIN_MISTER)/,lib/libco lib/miniz lib/md5 lib/lzma lib/zstd/lib lib/libchdr/include lib/bluetooth lib/serial_server/library)

MNU_CFLAGS = $(MNU_HOST_INCLUDES) -I. $(MNU_INCLUDES) -O2 -Wall -Wextra -Wno-psabi \
             -fPIC -fvisibility=hidden -fno-gnu-unique \
             -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE -DMNU_MODULE_BUILD -MMD -MP \
             $(MNU_CFLAGS_EXTRA)

MNU_OBJ = $(patsubst %.cpp,$(MNU_BUILD)/%.cpp.o,$(filter %.cpp,$(MNU_SRC))) \
          $(patsubst %.c,$(MNU_BUILD)/%.c.o,$(filter %.c,$(MNU_SRC)))

.PHONY: all clean deploy reload
all: $(MNU_OUT)

$(MNU_OUT): $(MNU_OBJ)
	$(CC) -shared -Wl,--no-undefined -o $@ $(MNU_OBJ) -lstdc++ -lc
	@$(READELF) --dyn-syms $@ | awk '$$5=="GLOBAL" && $$7!="UND" && $$8!~/^_/ && $$8!="mnu_entry" {print "warning: " FILENAME " exports " $$8}' FILENAME=$@; true

$(MNU_BUILD)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	$(CC) $(MNU_CFLAGS) -std=gnu++14 -c -o $@ $<

$(MNU_BUILD)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(MNU_CFLAGS) -std=gnu99 -c -o $@ $<

-include $(MNU_OBJ:.o=.d)

deploy: $(MNU_OUT)
	@test -n "$(MNU_SD_DIR)" || { echo "set MNU_SD_DIR to the card directory that holds the rbf"; exit 1; }
	scp $(MNU_OUT) root@$(MNU_HOST):$(MNU_SD_DIR)/$(MNU_SD_NAME)

reload:
	ssh root@$(MNU_HOST) 'echo mnu_reload > /dev/MiSTer_cmd'

clean:
	rm -rf $(MNU_BUILD) $(MNU_OUT)
