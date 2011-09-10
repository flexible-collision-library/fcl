all: installed

#
# Download, extract and compile from a released tarball:
#
TARBALL = build/svm_light.tar.gz
TARBALL_URL = http://download.joachims.org/svm_light/current/svm_light.tar.gz
TARBALL_PATCH = svm_light.diff
UNPACK_CMD = mkdir svm_light; tar -C svm_light -xzf
INITIAL_DIR = build/svm_light
SOURCE_DIR = build/svm_light-tar
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = svm_light
CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE="Release" -D CMAKE_INSTALL_PREFIX=`rospack find svm_light`/$(INSTALL_DIR) 
MAKE = make

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && make libsvmlight_hideo $(ROS_PARALLEL_JOBS)
	mkdir -p $(INSTALL_DIR)/lib
	mkdir -p $(INSTALL_DIR)/include
	mkdir -p $(INSTALL_DIR)/include/svm_light
	cp -r $(SOURCE_DIR)/*.h $(INSTALL_DIR)/include/svm_light
	cp -r $(SOURCE_DIR)/*.so $(INSTALL_DIR)/lib
	touch installed

clean:
	rm -rf build
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched
