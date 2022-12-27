CXX = g++
CXXFLAGS = -std=c++11
LDFLAGS = -lrealsense2
LDFLAGS += `pkg-config --libs opencv4`
CPPFLAGS = `pkg-config --cflags opencv4`

BGR_sample:
	$(CXX) $(CXXFLAGS) BGR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o BGR_sample

IR_sample:
	$(CXX) $(CXXFLAGS) IR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o IR_sample

stream:
	$(CXX) $(CXXFLAGS) stream.cpp $(LDFLAGS) $(CPPFLAGS) -o stream

april_depth:
	$(CXX) $(CXXFLAGS) april_depth.cpp $(LDFLAGS) -lapriltag $(CPPFLAGS) -o april_depth


.PHONY: BGR_sample IR_sample stream april_depth
