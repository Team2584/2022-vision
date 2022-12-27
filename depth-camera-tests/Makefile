CXX = g++
CXXFLAGS = -std=c++11
LDFLAGS = -lrealsense2
LDFLAGS += `pkg-config --libs opencv4`
CPPFLAGS = `pkg-config --cflags opencv4`

all: BGR_sample IR_sample

BGR_sample:
	$(CXX) $(CXXFLAGS) BGR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o BGR_sample

IR_sample:
	$(CXX) $(CXXFLAGS) IR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o IR_sample
