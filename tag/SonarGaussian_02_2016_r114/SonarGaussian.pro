TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += GroundTruth \
    Segmentation \
    Drawing

SOURCES += main.cpp \
    GraphMatcher/GraphMatcher.cpp \
    Cronometer.cpp \
    Segmentation/Segment.cpp \
    Segmentation/Segmentation.cpp \
    Drawing/Drawing.cpp \
    GroundTruth/GroundTruth.cpp \
    MatchViewer.cpp \
    GroundTruth/MatchHandler.cpp \
    Moisaic/Mosaic.cpp \
    GroundTruth/SonarDescritorTester.cpp \
#    MatchTest/MatchTest.cpp \
    WindowTool/WindowTool.cpp \
    WindowTool/WindowFeature.cpp \
    Sonar/Sonar.cpp \
    Sonar/SonarDescritor.cpp \
    Sonar/GraphLink.cpp \
    Sonar/GaussianTest.cpp \
    Sonar/Gaussian.cpp \
    Sonar/SonarConfig.cpp \
    WindowTool/SonarTestWindow.cpp \
    WindowTool/WindowFeatureTest.cpp \
    WindowTool/WFGroudTruth.cpp \
    WindowTool/WFTopologicalMatch.cpp \
    GroundTruth/FrameGT.cpp \
    WindowTool/Frame.cpp \
    WindowTool/FrameSD.cpp \
    GraphMatcher/GMPrincipalComponent.cpp \
    GraphMatcher/GMScalene.cpp \
    GraphMatcher/VertexMatcher.cpp \
    GraphMatcher/VMPrincipalComponent.cpp \
    GraphMatcher/GMBrutalForce.cpp \
    GraphMatcher/FilterMatch.cpp \
    GraphMatcher/SIFTMatchFilter.cpp \
    GraphMatcher/MatchInfoExtended.cpp \
    GraphMatcher/MatchInfo.cpp \
    Drawing/Chart.cpp \
    GraphMatcher/MatchInfoWeighted.cpp \
    CSVMatrixGenerator.cpp \
    CloseLoopTester.cpp \
    CloseLoopAnaliseResult.cpp \
    GraphMatcher/EdgeMatchInfo.cpp

LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
## LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/

HEADERS += \
    GraphMatcher/GraphMatcher.h \
    Cronometer.h \
    Enuns.h \
    Segmentation/Segment.h \
    Segmentation/Segmentation.h \
    Drawing/Drawing.h \
    GroundTruth/GroundTruth.h \
    MatchViewer.h \
    GroundTruth/MatchHandler.h \
    Moisaic/Mosaic.h \
    GroundTruth/SonarDescritorTester.h \
#    MatchTest/MatchTest.h \
    WindowTool/WindowTool.h \
    WindowTool/WindowFeature.h \
    Sonar/Sonar.h \
    Sonar/GraphLink.h \
    Sonar/SonarDescritor.h \
    Sonar/Gaussian.h \
    Sonar/GaussianTest.h \
    Sonar/SonarConfig.h \
    WindowTool/SonarTestWindow.h \
    WindowTool/WindowFeatureTest.h \
    WindowTool/WFGroudTruth.h \
    WindowTool/WFTopologicalMatch.h \
    WindowTool/Frame.h \
    GroundTruth/FrameGT.h \
    WindowTool/FrameSD.h \
    GraphMatcher/GMPrincipalComponent.h \
    GraphMatcher/GMScalene.h \
    GraphMatcher/VertexMatcher.h \
    GraphMatcher/VMPrincipalComponent.h \
    GraphMatcher/GMBrutalForce.h \
    GraphMatcher/FilterMatch.h \
    GraphMatcher/SIFTMatchFilter.h \
    GraphMatcher/MatchInfoExtended.cpp \
    GraphMatcher/MatchInfo.cpp \
    Drawing/Chart.h \
    GraphMatcher/MatchInfoWeighted.h \
    CSVMatrixGenerator.h \
    CloseLoopTester.h \
    CloseLoopAnaliseResult.h \
    GraphMatcher/EdgeMatchInfo.h

OTHER_FILES += \
    MachadosConfig \
    Sonar/MachadosConfig
