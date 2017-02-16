#ifndef CSVDATA_H
#define CSVDATA_H

#include <vector>
#include <cstdarg>
#include "CSVReader.h"

class CSVData
{
private:
    std::vector<CSV_TYPE> m_types;
    std::vector< std::vector<void *> > m_values;

    unsigned m_currentMsgID;

    CSVData();
public:
    ~CSVData();

    unsigned fieldCount();
    unsigned msgCount();

    CSV_TYPE fieldType(unsigned fieldID);

    int getField(unsigned fieldID, void * value);
    int getField(unsigned msgID, unsigned fieldID, void *value);

    void getMsg(CSV_MSG msgInfo, ...);


    void clearMsgs();

    bool nextMsg();
    void toBegin();

    friend class CSVReader;
};

#endif // CSVDATA_H
