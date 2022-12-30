CC = gcc
CXX = g++

CPPFLAGS = -I/usr/include/opencv4/
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O3
CXXFLAGS = -g -Wall -O3
LDFLAGS = -lpthread -lm -lapriltag

TARGETS := apriltag_demo opencv_demo

.PHONY: all
all: logi_pose clean

logi_pose: logi_pose.o
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) -L/usr/lib/aarch64-linux-gnu -lopencv_dnn -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core

%.o: %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
