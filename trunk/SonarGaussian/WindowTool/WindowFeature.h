#ifndef WINDOWFEATURE_H
#define WINDOWFEATURE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "WindowTool/Frame.h"

using namespace std;
using namespace cv;

class WindowTool;

class WindowFeature
{
protected:

    WindowTool *wt;

    friend class WindowTool;
public:
    WindowFeature();
    virtual ~WindowFeature();


    virtual void start();

    /**
     * @brief This method is used to create new
     * frames by WindowTool. Children class have
     * to override this method if its need to use
     * a different kind of frame.
     *
     * @param fileName - Image file name of new frame
     * @param frameNumber - The number (Identificaton)
     *  of new frame.
     */
    virtual Frame* newFrame(const string &fileName, unsigned frameNumber);


    /**
     * @brief This method is called every time that
     * a new is selected by mouse.
     *
     * @param frameId - Id of the selected frame.
     */
    virtual void selectedFrame(int frameId);

    /**
     * @brief This method is called when some key
     * is pressed. Children class have to override
     * this method to do something when a key is pressed.
     *
     * @param c
     */
    virtual void keyPress(char c);


    virtual void mouseEvent(int event, int x, int y);

    /**
     * @brief This method is called before show
     * imagens on screen and need to be overreded
     * by chieldren class to draw something on
     * images.
     *
     * @param leftImg - Color BGR left image.
     * @param leftId - Imagem ID.
     * @param rightImg - Color BGR right image.
     * @param rightId - Right image ID.
     */
    virtual void renderProcess(Mat &leftImg, int leftId,
                       Mat &rightImg, int rightId);


    /**
     * @brief This method is called before show
     * image on screen and after renderFrameTogether()
     * method. Children class have to implement this
     * method to draw something on screen.
     *
     *    Thet transformation parameters are four
     * float values:
     *  [0] - Width aspect ratio ( result width / original width )
     *  [1] - Height aspect ratio ( result heght / original hieght )
     *  [2] - Horizontal imagem position (column)
     *  [3] - Vertical imagem postion (row)
     * @param screen - BGR image that will be show on screen
     * after this method.
     * @param el - Transformatio parameters for lefht image.
     * @param leftFrameId
     * @param er - Transformation parameters for right image.
     * @param rightFrameId
     */
    virtual void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);


    /**
     * @brief This method is called when a new image
     * is loaded. Chieldren class have to overide this
     * method to do something when a new image is load.
     *
     * @param leftImg - new left imagem (gray 16 bits).
     * @param leftId - left image ID.
     * @param rightImg - new right image (gray 16 bits).
     * @param rightId - right image ID.
     */
    virtual void processImages(Mat &leftImg, int leftId,
                       Mat &rightImg, int rightId);

};

#endif // WINDOWFEATURE_H
