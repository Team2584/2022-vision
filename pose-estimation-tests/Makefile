CC = gcc
CXX = g++

CPPFLAGS = `pkg-config --cflags opencv4`
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O3
CXXFLAGS = -g -Wall -O3
LDFLAGS = -lpthread -lm -lapriltag

TARGETS := apriltag_demo opencv_demo

.PHONY: all
all: logi_pose

logi_pose: logi_pose.o
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv4`

%.o: %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
