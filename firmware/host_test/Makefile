# Host-side tests for firmware code that does not need the board.
#   make -C firmware/host_test           cal_store.h power-cut test (seconds)
#   make -C firmware/host_test littlefs  why calibration left LittleFS (~1 min)
CXX ?= g++
CC ?= gcc
SAN = -fsanitize=address,undefined -fno-sanitize-recover=all
LFS = $(HOME)/.platformio/packages/framework-arduinopico/libraries/LittleFS/lib/littlefs
BUILD ?= build

test: $(BUILD)/cal_store_test
	$<

littlefs: $(BUILD)/littlefs_powercut
	$< 200 16

$(BUILD)/cal_store_test: cal_store_test.cpp ../src/cal_store.h
	@mkdir -p $(BUILD)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra $(SAN) $< -o $@

$(BUILD)/littlefs_powercut: littlefs_powercut.c
	@mkdir -p $(BUILD)
	$(CC) -O1 -g -w $(SAN) -I$(LFS) -DLFS_C='"$(LFS)/lfs.c"' $< $(LFS)/lfs_util.c -o $@

.PHONY: test littlefs
