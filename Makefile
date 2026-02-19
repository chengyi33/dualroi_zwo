CXX = g++
CXXFLAGS = -D_LIN -O2
INCLUDES = -I./include
LIBS = -L./lib/x64 -lASICamera2 -lpthread -lrt
OPENCV = $(shell pkg-config --cflags --libs opencv4)
RPATH = -Wl,-rpath,'$$ORIGIN/lib/x64'

TARGET = bin/dual_roi
SRC = src/dual_roi.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(INCLUDES) $(LIBS) $(OPENCV) $(RPATH)
	@echo "Build complete: $(TARGET)"

clean:
	rm -f $(TARGET)

install-udev:
	@echo "Installing udev rules for ZWO cameras (requires sudo)..."
	sudo cp udev/asi.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger
	@echo "Done. Unplug and replug your camera."

run: $(TARGET)
	LD_LIBRARY_PATH=./lib/x64 ./$(TARGET)

.PHONY: all clean install-udev run
