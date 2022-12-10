all: bin

bin:
	rm -rf build
	mkdir -p build
	cd build && cmake ..
	cd build && make
	mv build/video_rcver .
	rm -rf build

clean:
	rm -rf build video_rcver

