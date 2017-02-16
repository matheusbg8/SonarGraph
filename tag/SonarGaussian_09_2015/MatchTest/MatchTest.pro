TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += GroundTruth \
    Segmentation \
    Drawing

SOURCES += main.cpp \ 
    GraphLink.cpp \
    Gaussian.cpp \
    SonarDescritor.cpp \
    SpatialMatchInfo.cpp

LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
## LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/

HEADERS += \
    GraphLink.h \
    Gaussian.h \
    SonarDescritor.h \
    SpatialMatchInfo.h

