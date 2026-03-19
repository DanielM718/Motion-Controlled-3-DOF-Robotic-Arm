COMPILER := g++

BUILD := build/main.out

INCLUDE := $(shell find include -type f -name "*.h")
SRC := $(shell find src -type f -name "*.cpp")

all: run

run: $(INCLUDE) $(SRC)
	mkdir -p build
	$(COMPILER) -Iinclude -o $(BUILD) $(SRC)

clean:
	rm -rf build