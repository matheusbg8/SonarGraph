TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#LIBS += `pkg-config --cflags --libs opencv` -L/usr/local/lib/
#LIBS += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -I /usr/local/lib/


#LIBS += -Wl,-rpath,/home/matheusbg/opencv-2.4.13/build_qt -L/home/matheusbg/opencv-2.4.13/build_qt -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy
#LIBS += -Wl,-rpath,/home/matheusbg/opencv-2.4.13/build -L/home/matheusbg/opencv-2.4.13/build -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy

#LIBS += -Wl,-rpath,/home/matheusbg8/Downloads/opencv-2.4.10/build/lib -L/home/matheusbg8/Downloads/opencv-2.4.10/build/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy
LIBS += -Wl,-rpath,/home/matheusbg/Downloads/opencv-2.4.11/build/lib -L/home/matheusbg/Downloads/opencv-2.4.11/build/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy

LIBS += -lboost_system -lboost_thread -lboost_filesystem

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
    WindowTool/SonarTestWindow.cpp \
    WindowTool/WindowFeatureTest.cpp \
    WindowTool/WFGroudTruth.cpp \
    GroundTruth/FrameGT.cpp \
    WindowTool/Frame.cpp \
    WindowTool/FrameSD.cpp \
    GraphMatcher/MatchInfo/MatchInfo.cpp \
    GraphMatcher/MatchInfo/MatchInfoExtended.cpp \
    GraphMatcher/MatchInfo/MatchInfoWeighted.cpp \
    Drawing/Chart.cpp \
    Sonar/SonarConfig/Automaton.cpp \
    Sonar/SonarConfig/AutomatonEdge.cpp \
    Segmentation/SegmentExtractor/SegmentExtractor.cpp \
    Segmentation/SegmentExtractor/FullSegmentExtractor.cpp \
    Segmentation/SegmentExtractor/BorderSegmentExtractor.cpp \
    Segmentation/SegmentExtractor/DistantSegmentExtractor.cpp \
    Segmentation/SegmentExtractor/RelativeSegmentExtractor.cpp \
    Segmentation/SegmentExtractor/PixelRelativeSegmentExtractor.cpp \
    Segmentation/SegmentSearcher/SegmentSearcher.cpp \
    Segmentation/SegmentSearcher/LinearSegmentSearcher.cpp \
    Segmentation/SegmentSearcher/DoubleSegmentSearcher.cpp \
    Segmentation/SegmentSearcher/SegmentWithPeak.cpp \
    GraphMatcher/VertexMatcher/VertexMatcher.cpp \
    GraphMatcher/VertexMatcher/VMBrutalForce.cpp \
    GraphMatcher/VertexMatcher/VMWeghtedSumPC.cpp \
    GraphMatcher/VertexMatcher/VMScalenePC.cpp \
    GraphMatcher/GraphMatchFinder/GraphMatchFinder.cpp \
    GraphMatcher/GraphMatchFinder/GMFVertexByVertex.cpp \
    Sonar/SonarConfig/ConfigLoader.cpp \
    GraphMatcher/VertexMatcher/VMHeuristicByAngVariation1.cpp \
    GraphMatcher/VertexMatcher/VMHeuristicByAngVariation.cpp \
    CloseLoopTester.cpp \
    CloseLoopAnaliseResult.cpp \
    System/Files.cpp \
    CSVMatrixGenerator.cpp \
    WindowTool/SVM/WFSVM.cpp \
    WindowTool/GaussianDescriptor/WFGaussianDescriptor.cpp \
    WindowTool/GaussianDescriptor/GaussianFrame.cpp \
    WindowTool/GaussianDescriptor/GausianDescriptorFeature.cpp \
    WindowTool/SVM/SVMFrame.cpp \
    WindowTool/WindowPaint.cpp \
    WindowTool/PaintFeatures/PaintFeature.cpp \
    WindowTool/PaintFeatures/ImageMoments.cpp \
    WindowTool/SingleImageWindow.cpp \
    Drawing/FrameRender/FrameRender.cpp \
    Drawing/FrameRender/TextInfo.cpp \
    WindowTool/GaussianDescriptor/GaussianRender.cpp \
    Drawing/FrameRender/ObjectRender.cpp \
    WindowTool/GaussianDescriptor/GaussianFrameRender.cpp \
    WindowTool/SingleImageWindow/SingleImageFeature.cpp \
    WindowTool/SingleImageWindow/TestSVMFeature.cpp \
    WindowTool/GTLoop/WFGTLoop.cpp \
    WindowTool/GTLoop/GTLoopFrame.cpp \
    WindowTool/GTLoop/GTLoopLoader.cpp \
    RoboMap/RoboMap.cpp \
    RoboMap/RoboMapElement.cpp \
    RoboMap/RoboMapSonar.cpp \
    GraphMatcher/VertexMatcher/VMbyEdgeLength.cpp \
    Tools/HungarianAlgorithm.cpp \
    GraphMatcher/GraphMatchFinder/GMFHungarian.cpp \
    Tools/IndexMapping.cpp \
    WindowTool/TopologicalVertexMatch/WFTopologicalVertexMatch.cpp \
    WindowTool/TopologicalMatch/WFTopologicalMatch.cpp \
    GraphMatcher/GraphMatchFinder/GMFByVertexMatch.cpp \
    Segmentation/SegmentSearcher/ThetaRhoSortSegSearch.cpp \
    Segmentation/SegmentSearcher/ThetaRhoSegmentSearcher.cpp \
    Segmentation/SegmentSearcher/ThetaRhoMeanPeakSegSearch.cpp \
    Tools/CircularQueue.cpp \
    GraphMatcher/GraphMatchFinder/GMFByEdgeExploration.cpp \
    GraphMatcher/GraphMatchFinder/GMFBestDirect.cpp \
    Tools/ModelEvaluation.cpp \
    Tools/GenericImageProcessing.cpp \
    Tools/DirOperations.cpp \
    Segmentation/SegmentExtractor/DistantSegmentExtractorV2.cpp \
    RoboMap/MapPloter.cpp \
    CSVReader/CSVReader2.cpp \
    CSVReader/CSVData.cpp \
    CSVReader/CSVReader.cpp \
    CSVReader/DataExtractorUtilities.cpp



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
    WindowTool/SonarTestWindow.h \
    WindowTool/WindowFeatureTest.h \
    WindowTool/WFGroudTruth.h \
    WindowTool/Frame.h \
    GroundTruth/FrameGT.h \
    WindowTool/FrameSD.h \
    GraphMatcher/MatchInfo/MatchInfo.h \
    GraphMatcher/MatchInfo/MatchInfoExtended.h \
    GraphMatcher/MatchInfo/MatchInfoWeighted.h \
    Drawing/Chart.h \
    Sonar/SonarConfig/Automaton.h \
    Sonar/SonarConfig/AutomatonEdge.h \
    Segmentation/SegmentExtractor/SegmentExtractor.h \
    Segmentation/SegmentExtractor/FullSegmentExtractor.h \
    Segmentation/SegmentExtractor/BorderSegmentExtractor.h \
    Segmentation/SegmentExtractor/DistantSegmentExtractor.h \
    Segmentation/SegmentExtractor/RelativeSegmentExtractor.h \
    Segmentation/SegmentExtractor/PixelRelativeSegmentExtractor.h \
    Segmentation/SegmentSearcher/SegmentSearcher.h \
    Segmentation/SegmentSearcher/LinearSegmentSearcher.h \
    Segmentation/SegmentSearcher/DoubleSegmentSearcher.h \
    Segmentation/SegmentSearcher/SegmentWithPeak.h \
    Segmentation/SegmentSearcher/ThetaRhoSegmentSearcher.h \
    GraphMatcher/VertexMatcher/VertexMatcher.h \
    GraphMatcher/VertexMatcher/VMBrutalForce.h \
    GraphMatcher/VertexMatcher/VMWeghtedSumPC.h \
    GraphMatcher/VertexMatcher/VMScalenePC.h \
    GraphMatcher/VertexMatcher/VertexMatcher.h \
    GraphMatcher/GraphMatchFinder/GraphMatchFinder.h \
    Sonar/SonarConfig/ConfigLoader.h \
    GraphMatcher/VertexMatcher/VMHeuristicByAngVariation1.h \
    GraphMatcher/VertexMatcher/VMHeuristicByAngVariation.h \
    CloseLoopTester.h \
    CloseLoopAnaliseResult.h \
    System/Files.h \
    CSVMatrixGenerator.h \
    WindowTool/SVM/WFSVM.h \
    WindowTool/GaussianDescriptor/WFGaussianDescriptor.h \
    WindowTool/GaussianDescriptor/GaussianFrame.h \
    WindowTool/GaussianDescriptor/GausianDescriptorFeature.h \
    WindowTool/SVM/SVMFrame.h \
    WindowTool/WindowPaint.h \
    WindowTool/PaintFeatures/PaintFeature.h \
    WindowTool/PaintFeatures/ImageMoments.h \
    WindowTool/SingleImageWindow.h \
    Drawing/FrameRender/FrameRender.h \
    WindowTool/GaussianDescriptor/GaussianRender.h \
    Drawing/FrameRender/ObjectRender.h \
    WindowTool/GaussianDescriptor/GaussianFrameRender.h \
    Drawing/FrameRender/TextRender.h \
    WindowTool/SingleImageWindow/SingleImageFeature.h \
    WindowTool/SingleImageWindow/TestSVMFeature.h \
    WindowTool/GTLoop/WFGTLoop.h \
    WindowTool/GTLoop/GTLoopFrame.h \
    WindowTool/GTLoop/GTLoopLoader.h \
    RoboMap/RoboMap.h \
    RoboMap/RoboMapElement.h \
    RoboMap/RoboMapSonar.h \
    GraphMatcher/VertexMatcher/VMbyEdgeLength.h \
    Tools/HungarianAlgorithm.h \
    GraphMatcher/GraphMatchFinder/GMFVertexByVertex.h \
    GraphMatcher/GraphMatchFinder/GMFHungarian.h \
    Tools/IndexMapping.h \
    WindowTool/TopologicalVertexMatch/WFTopologicalVertexMatch.h \
    WindowTool/TopologicalMatch/WFTopologicalMatch.h \
    GraphMatcher/GraphMatchFinder/GMFByVertexMatch.h \
    Segmentation/SegmentSearcher/ThetaRhoSortSegSearch.h \
    Segmentation/SegmentSearcher/ThetaRhoMeanPeakSegSearch.h \
    Tools/CircularQueue.h \
    GraphMatcher/GraphMatchFinder/GMFByEdgeExploration.h \
    GraphMatcher/GraphMatchFinder/GMFBestDirect.h \
    Tools/ModelEvaluation.h \
    Tools/GenericImageProcessing.h \
    Tools/DirOperations.h \
    Segmentation/SegmentExtractor/DistantSegmentExtractorV2.h \
    RoboMap/MapPloter.h \
    CSVReader/CSVReader2.h \
    CSVReader/CSVData.h \
    CSVReader/CSVReader.h \
    CSVReader/DataExtractorUtilities.h

OTHER_FILES += \
    MachadosConfig \
    Sonar/MachadosConfig \
    Configs.ini \
    Sonar/Anotations.txt
