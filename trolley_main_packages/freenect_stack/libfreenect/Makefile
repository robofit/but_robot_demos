all: libfreenect

libfreenect: 
	git clone git://github.com/OpenKinect/libfreenect
	cd libfreenect/ && git checkout unstable
	cd libfreenect/ && mkdir build
	cd libfreenect/build/ && cmake -DBUILD_AUDIO=YES ..
	cd libfreenect/build/ &&make
	cp -rf libfreenect/include include
	cp -rf libfreenect/build/bin bin
	cp -rf libfreenect/build/lib lib
	touch libfreenect

clean:
	-rm -rf src libfreenect include bin lib

wipe: clean
	-rm -rf build
