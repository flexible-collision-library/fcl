all:	
	mkdir -p build
	cd build && cmake .. && make

clean:
	-cd build && make clean
	rm -rf build

test: all
	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi


