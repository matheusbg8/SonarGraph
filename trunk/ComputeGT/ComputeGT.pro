TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
## LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/

LIBS += -lCGAL -lgmp -frounding-math -lboost_thread -lboost_system -lpthread -lmpfr

SOURCES += main.cpp \
    GTLoop.cpp \
    RoboMap/RoboMap.cpp \
    RoboMap/RoboMapElement.cpp \
    RoboMap/RoboMapSonar.cpp \
    GTLoopFrame.cpp \
    GTLoopLoader.cpp \
    ConfigLoader/Automaton.cpp \
    ConfigLoader/AutomatonEdge.cpp \
    ConfigLoader/ConfigLoader.cpp \
    ConfigLoader/DataExtractorUtilities.cpp \
    CGALDef.cpp

HEADERS += \
    GTLoop.h \
    RoboMap/RoboMap.h \
    RoboMap/RoboMapElement.h \
    RoboMap/RoboMapSonar.h \
    GTLoopFrame.h \
    GTLoopLoader.h \
    ConfigLoader/Automaton.h \
    ConfigLoader/AutomatonEdge.h \
    ConfigLoader/ConfigLoader.h \
    ConfigLoader/DataExtractorUtilities.h \
    CGALDef.h

