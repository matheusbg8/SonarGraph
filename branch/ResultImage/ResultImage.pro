TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
#LIBS += -Wl,-rpath,/home/matheusbg/Downloads/opencv-2.4.11/build/lib -L/home/matheusbg/Downloads/opencv-2.4.11/build/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy
#LIBS += -Wl,-rpath,/home/matheusbg/opencv-2.4.13/build/lib -L/home/matheusbg/opencv-2.4.13/build/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy

LIBS += -lboost_system -lboost_thread -lboost_filesystem

SOURCES += main.cpp \
    Compare.cpp

HEADERS += \
    Compare.h

