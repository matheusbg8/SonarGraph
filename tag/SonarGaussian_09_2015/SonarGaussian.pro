TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += GroundTruth \
    Segmentation \
    Drawing

SOURCES += main.cpp \
    Sonar.cpp \
    Gaussian.cpp \
    GraphLink.cpp \
    SonarDescritor.cpp \
    GraphMatcher/GraphMatcher.cpp \
    Cronometer.cpp \
    SonarConfig.cpp \
    Segmentation/Segment.cpp \
    Segmentation/Segmentation.cpp \
    Drawing/Drawing.cpp \
    GroundTruth/GroundTruth.cpp \
    GroundTruth/Frame.cpp \
    MatchViewer.cpp \
    GroundTruth/MatchHandler.cpp \
    Moisaic/Mosaic.cpp \
    GroundTruth/SonarDescritorTester.cpp \
    SpatialMatchInfo.cpp \
#    MatchTest/MatchTest.cpp \
    GaussianTest.cpp

LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
## LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/

HEADERS += \
    Sonar.h \
    Gaussian.h \
    GraphLink.h \
    SonarDescritor.h \
    GraphMatcher/GraphMatcher.h \
    Cronometer.h \
    SonarConfig.h \
    Enuns.h \
    Segmentation/Segment.h \
    Segmentation/Segmentation.h \
    Drawing/Drawing.h \
    GroundTruth/Frame.h \
    GroundTruth/GroundTruth.h \
    MatchViewer.h \
    GroundTruth/MatchHandler.h \
    Moisaic/Mosaic.h \
    GroundTruth/SonarDescritorTester.h \
    SpatialMatchInfo.h \
#    MatchTest/MatchTest.h \
    GaussianTest.h

OTHER_FILES += \
    MachadosConfig
