all: bin

bin:
	rm -rf build
	mkdir -p build
	cd build && cmake ..
	cd build && make
	mv build/video_server .
	rm -rf build

clean:
	rm -rf build video_server

