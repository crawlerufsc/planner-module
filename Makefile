all: bin

lib:
	cd ../control/lib && $(MAKE) all

bin:
	rm -rf build
	mkdir -p build
	cd build && cmake ..
	cd build && make -j$(nproc)
	mv build/planner-bin .


clean:
	rm -rf build
	rm planner-bin

