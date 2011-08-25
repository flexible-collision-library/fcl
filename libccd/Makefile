all: installed

#
# Download, extract and compile from a released tarball:
#
TARBALL = build/libccd_1.0.tar.gz
TARBALL_URL = http://libccd.danfis.cz/files/libccd-1.0.tar.gz
TARBALL_PATCH = fPIC.diff
INITIAL_DIR = build/libccd-1.0
SOURCE_DIR = build/libccd-tar
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = libccd
CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE="Release" -D CMAKE_INSTALL_PREFIX=`rospack find libccd`/$(INSTALL_DIR) 
MAKE = make

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)/src && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(INSTALL_DIR)/lib
	mkdir -p $(INSTALL_DIR)/include
	mkdir -p $(INSTALL_DIR)/include/ccd
	cp -r $(SOURCE_DIR)/src/ccd/*.h $(INSTALL_DIR)/include/ccd
	cp -r $(SOURCE_DIR)/src/*.a $(INSTALL_DIR)/lib
	touch installed

clean:
	rm -rf build
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched
