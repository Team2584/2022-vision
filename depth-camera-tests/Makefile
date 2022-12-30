CXX = g++
CXXFLAGS = -std=c++11
LDFLAGS = -lrealsense2
LDFLAGS += -L/usr/lib/aarch64-linux-gnu
# LDFLAGS += -lopencv_fuzzy
# LDFLAGS += -lopencv_calib3d
# LDFLAGS += -lopencv_ml
# LDFLAGS += -lopencv_dpm
# LDFLAGS += -lopencv_cudev
# LDFLAGS += -lopencv_cudabgsegm
# LDFLAGS += -lopencv_cudafilters
# LDFLAGS += -lopencv_stitching
LDFLAGS += -lopencv_videoio
# LDFLAGS += -lopencv_dnn_objdetect
LDFLAGS += -lopencv_rgbd
# LDFLAGS += -lopencv_cudaimgproc
# LDFLAGS += -lopencv_cudastereo
# LDFLAGS += -lopencv_cudalegacy
# LDFLAGS += -lopencv_flann
# LDFLAGS += -lopencv_cudaobjdetect
# LDFLAGS += -lopencv_cudacodec
# LDFLAGS += -lopencv_imgcodecs
# LDFLAGS += -lopencv_cudaarithm
# LDFLAGS += -lopencv_gapi
# LDFLAGS += -lopencv_videostab
LDFLAGS += -lopencv_video
LDFLAGS += -lopencv_imgproc
# LDFLAGS += -lopencv_cudafeatures2d
# LDFLAGS += -lopencv_photo
LDFLAGS += -lopencv_core
LDFLAGS += -lopencv_plot
# LDFLAGS += -lopencv_objdetect
# LDFLAGS += -lopencv_dnn
LDFLAGS += -lopencv_highgui
# LDFLAGS += -lopencv_cudawarping
# LDFLAGS += -lopencv_features2d

CPPFLAGS = -I/usr/include/opencv4

BGR_sample:
	$(CXX) $(CXXFLAGS) BGR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o BGR_sample

IR_sample:
	$(CXX) $(CXXFLAGS) IR_sample.cpp $(LDFLAGS) $(CPPFLAGS) -o IR_sample

stream:
	$(CXX) $(CXXFLAGS) stream.cpp $(LDFLAGS) $(CPPFLAGS) -o stream

april_depth:
	$(CXX) $(CXXFLAGS) april_depth.cpp $(LDFLAGS) -lapriltag $(CPPFLAGS) -o april_depth

get_camera_props:
	$(CXX) $(CXXFLAGS) get_camera_props.cpp $(LDFLAGS) $(CPPFLAGS) -o get_camera_props


.PHONY: BGR_sample IR_sample stream april_depth get_camera_props
