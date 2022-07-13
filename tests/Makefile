all: bin

bin:
	rm -rf build
	mkdir -p build
	cd build && cmake ..
	cd build && make
	mv build/planner-tst .
	rm -rf build

clean:
	rm -rf build
	rm planner-tst

test: all
	./planner-tst
	