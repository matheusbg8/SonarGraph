#ifndef FRAME_H
#define FRAME_H

#include <string>

using namespace std;

/**
 * @brief
 *
 */
class Frame
{
public:
    string fileName;
    unsigned frameNumber;

    Frame(const string& fileName,unsigned frameNumber);
    virtual ~Frame();

};

ostream & operator << (ostream&os , const Frame& fr);

#endif // FRAME_H
