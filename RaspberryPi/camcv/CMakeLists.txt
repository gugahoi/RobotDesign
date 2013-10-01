cmake_minimum_required(VERSION 2.8)
project( camcv )
SET(COMPILE_DEFINITIONS -Werror)
find_package( OpenCV REQUIRED )

include_directories(/opt/vc/userland/host_applications/linux/libs/bcm_host/include)
include_directories(/opt/vc/userland/interface/vcos)
include_directories(/opt/vc/userland)
include_directories(/opt/vc/userland/interface/vcos/pthreads)
include_directories(/opt/vc/userland/interface/vmcs_host/linux)
include_directories(/root/Desktop/robot/gnublin-api)
include_directories(${OpenCV_INCLUDE_DIR})

add_executable(camcv ./External/gnublin.cpp arduino_ctrl.cpp RaspiCamControl.c RaspiCLI.c RaspiPreview.c main.cpp)

target_link_libraries(camcv /root/Desktop/robot/gnublin-api/libgnublin.so /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so ${OpenCV_LIBS})
