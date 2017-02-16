TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/

SOURCES += main.cpp \
    Frame.cpp \
    GroundTruth.cpp \
    Gaussian/Gaussian.cpp \
    Segmentation/Segmentation.cpp \
    Segmentation/Segment.cpp

HEADERS += \
    Frame.h \
    GroundTruth.h \
    Gaussian/Gaussian.h \
    Segmentation/Segmentation.h \
    Segmentation/Segment.h \
    Gaussian/Gaussian.h

