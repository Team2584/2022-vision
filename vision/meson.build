project('vision', 'cpp', default_options : ['cpp_std=c++20'])

cc = meson.get_compiler('cpp')

# Opencv
opencv_dirs = ['/usr/lib/aarch64-linux-gnu']
opencv_deps = [
               cc.find_library('opencv_core', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_calib3d', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_videoio', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_rgbd', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_video', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_imgproc', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_plot', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_highgui', required: true, dirs: opencv_dirs),
              ]
opencv_inc = include_directories('/usr/include/opencv4')

# Wpi
wpi_deps = [
            cc.find_library('wpinet', required: true),
            cc.find_library('ntcore', required: true),
           ]

wpi_incs = [
            include_directories('/usr/local/include/ntcore/'),
            include_directories('/usr/local/include/wpiutil/'),
            include_directories('/usr/local/include/wpinet/'),
           ]

# THE REAL STUFF

src = ['main.cpp']

deps = [
        opencv_deps,
        dependency('realsense2'),
        dependency('eigen3'),
        dependency('apriltag'),
        wpi_deps,
       ]

inc = [
       opencv_inc,
       wpi_incs,
      ]

executable('runvision',
           sources : src,
           dependencies : deps,
           include_directories : inc,
           cpp_args : ['-std=c++20', '-Wno-deprecated'])
