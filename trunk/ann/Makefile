all: installed

#
# Download, extract and compile from a released tarball:
#
TARBALL = build/ann_1.1.2.tar.gz
TARBALL_URL = http://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
TARBALL_PATCH = fANN.diff
INITIAL_DIR = build/ann_1.1.2
SOURCE_DIR = build/ann-tar
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = ann
CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE="Release" -D CMAKE_INSTALL_PREFIX=`rospack find ann`/$(INSTALL_DIR) 
MAKE_ARGS = linux-g++
MAKE = make

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && make $(ROS_PARALLEL_JOBS) $(MAKE_ARGS)
	mkdir -p $(INSTALL_DIR)/include
	mkdir -p $(INSTALL_DIR)/include/ann
	cp -r $(SOURCE_DIR)/include/ANN/*.h $(INSTALL_DIR)/include/ann
	cp -r $(SOURCE_DIR)/bin $(INSTALL_DIR)
	cp -r $(SOURCE_DIR)/lib $(INSTALL_DIR)
	touch installed

clean:
	rm -rf build
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched
