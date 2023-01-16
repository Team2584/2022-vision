################################################################################
# Acquisition Makefile
################################################################################
PROJECT_ROOT=.

################################################################################
# Key paths and settings
################################################################################
CFLAGS += -std=c++11
ifeq ($(wildcard ${OPT_INC}),)
CXX = g++ ${CFLAGS}
ODIR  = .obj/build${D}
SDIR  = .
MKDIR = mkdir -p
PLATFORM = $(shell uname)
ifeq ($(PLATFORM),Darwin)
OS = mac
endif
endif

OUTPUTNAME = flir-calib${D}

################################################################################
# Dependencies
################################################################################
# Spinnaker deps
SPINNAKER_LIB = -L/opt/spinnaker/lib -lSpinnaker${D} ${SPIN_DEPS}

################################################################################
# Master inc/lib/obj/dep settings
################################################################################
_OBJ = flir-calib.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
INC = -I/opt/spinnaker/include -I/usr/include/opencv4
LIB += -Wl,-Bdynamic ${SPINNAKER_LIB}
LIB += -Wl,-rpath-link=../../lib
LIB += -Wl,-Bdynamic -lapriltag
LIB += -L/usr/lib/aarch64-linux-gnu -lopencv_dnn -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core

################################################################################
# Rules/recipes
################################################################################
# Final binary
${OUTPUTNAME}: ${OBJ}
	${CXX} -o ${OUTPUTNAME} ${OBJ} ${LIB}

# Intermediate object files
${OBJ}: ${ODIR}/%.o : ${SDIR}/%.cpp
	@${MKDIR} ${ODIR}
	${CXX} ${CFLAGS} ${INC} -Wall -D LINUX -c $< -o $@

# Clean up intermediate objects
clean_obj:
	rm -f ${OBJ}
	@echo "intermediate objects cleaned up!"

# Clean up everything.
clean: clean_obj
	rm -f ${OUTPUTNAME}
	@echo "all cleaned up!"

intel:
	g++ -ggdb -std=c++20 -Wno-deprecated depth-calib.cpp -lrealsense2 -L/usr/lib/aarch64-linux-gnu -lopencv_calib3d -lopencv_videoio -lopencv_rgbd -lopencv_video -lopencv_imgproc -lopencv_core -lopencv_plot -lopencv_highgui -lwpinet -lntcore -lapriltag -I/usr/include/opencv4 -I/usr/local/include/wpiutil/ -I/usr/local/include/ntcore/ -I/usr/local/include/wpinet/ -I/usr/include/eigen3 -o depth-calib
