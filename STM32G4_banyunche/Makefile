.PHONY: all debug release clean

all: debug

debug:
	cmake --preset Debug
	cmake --build --preset Debug

release:
	cmake --preset Release
	cmake --build --preset Release

clean:
	cmake --build --preset Debug --target clean
