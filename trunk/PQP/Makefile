all: installed

#
# Download, extract and compile from a released tarball:
#
TARBALL = build/pqp-1.3.tar.gz
TARBALL_URL = http://gamma.cs.unc.edu/software/downloads/SSV/pqp-1.3.tar.gz
TARBALL_PATCH = pqp.diff
INITIAL_DIR = build/pqp-1.3
SOURCE_DIR = build/pqp-tar
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = PQP
CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE="Release" -D CMAKE_INSTALL_PREFIX=`rospack find PQP`/$(INSTALL_DIR) 
MAKE = make

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)/PQP_v1.3 && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(INSTALL_DIR)/lib
	mkdir -p $(INSTALL_DIR)/include
	cp -r $(SOURCE_DIR)/PQP_v1.3/include/*.h $(INSTALL_DIR)/include
	cp -r $(SOURCE_DIR)/PQP_v1.3/lib/*.a $(INSTALL_DIR)/lib
	touch installed

clean:
	rm -rf build
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched
