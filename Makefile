# Makefile for JTAC Coordinate System Plugin
# Cross-compilation for Windows using MinGW-w64

# Compiler settings
CC = x86_64-w64-mingw32-gcc
CXX = x86_64-w64-mingw32-g++

# Plugin name and output
PLUGIN_NAME = JTACCoords
OUTPUT_DIR = build
PLUGIN_DIR = $(OUTPUT_DIR)/$(PLUGIN_NAME)
PLUGIN_FILE = $(PLUGIN_DIR)/win_x64/$(PLUGIN_NAME).xpl

# SDK paths
SDK_DIR = SDK
INCLUDE_DIRS = -I$(SDK_DIR)/CHeaders/XPLM -I$(SDK_DIR)/CHeaders/Widgets -I.
LIB_DIR = $(SDK_DIR)/Libraries/Win
LIBS = -L$(LIB_DIR) -lXPLM_64 -lXPWidgets_64

# Compiler flags for C++
CXXFLAGS = -std=c++11 -Wall -O2 -fPIC -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1 -DXPLM400=1
CXXFLAGS += $(INCLUDE_DIRS)
CXXFLAGS += -DIBM=1 -DWIN32=1 -D_WIN32=1

# Linker flags for Windows DLL
LDFLAGS = -shared -static-libgcc -static-libstdc++
LDFLAGS += -Wl,--kill-at -Wl,--no-undefined
LDFLAGS += $(LIBS)
LDFLAGS += -lopengl32 -lgdi32 -lm

# Source files
SOURCES = probe.cpp missile_guidance.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: directories $(PLUGIN_FILE)

# Create necessary directories
directories:
	@mkdir -p $(PLUGIN_DIR)/win_x64

# Build the plugin
$(PLUGIN_FILE): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	@echo "JTAC Coordinate System Plugin built successfully: $@"

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS)
	rm -rf $(OUTPUT_DIR)

# Test compilation without linking
test-compile:
	$(CXX) $(CXXFLAGS) -c $(SOURCES)
	@echo "Compilation test successful"

# Development helpers
debug: CXXFLAGS += -g -DDEBUG=1
debug: directories $(PLUGIN_FILE)

release: CXXFLAGS += -DNDEBUG=1 -s
release: directories $(PLUGIN_FILE)

# Check MinGW installation
check-mingw:
	@which x86_64-w64-mingw32-gcc > /dev/null || (echo "MinGW-w64 not found. Install with: sudo apt-get install mingw-w64" && exit 1)
	@echo "MinGW-w64 found and ready for cross-compilation"

# Installation helper
install: $(PLUGIN_FILE)
	@echo "Plugin built at: $(PLUGIN_FILE)"
	@echo "Copy the entire $(PLUGIN_DIR) folder to your X-Plane 12/Resources/plugins/ directory"

.PHONY: all clean directories test-compile debug release check-mingw install

# Dependencies
probe.o: probe.cpp