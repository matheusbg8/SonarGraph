#include <CSVReader/CSVData.h>
#include <string>

CSVData::CSVData():
    m_currentMsgID(0u)
{

}

CSVData::~CSVData()
{
    clearMsgs();
}

unsigned CSVData::fieldCount()
{
    return m_types.size();
}

unsigned CSVData::msgCount()
{
    return m_values.size();
}

CSV_TYPE CSVData::fieldType(unsigned fieldID)
{
    if(fieldID < m_types.size())
    {
        return m_types[fieldID];
    }
    return CSV_INVALID;
}

int CSVData::getField(unsigned fieldID, void *value)
{
    return getField(m_currentMsgID, fieldID, value);
}

int CSVData::getField(unsigned msgID, unsigned fieldID, void *value)
{
    if(fieldID < m_types.size() && msgID < m_values.size())
    {
        switch(m_types[fieldID])
        {
            case CSV_STRING:
                *((std::string*) value) = *((std::string*)  m_values[msgID][fieldID]);
            break;
            case CSV_INT:
                *((int*) value) = *((int*) m_values[msgID][fieldID]);
            break;
            case CSV_LLU:
                *((long long unsigned int*) value) = *((long long unsigned int*) m_values[msgID][fieldID]);
            break;
            case CSV_FLOAT:
                *((float*) value) = *((float*) m_values[msgID][fieldID]);
            break;
            case CSV_DOUBLE:
                *((double*) value) = *((double*) m_values[msgID][fieldID]);
            break;
            case CSV_INVALID:
            default:
                // Nothing to do
            break;
        }
    }
    return 0;
}

void CSVData::getMsg(CSV_MSG msgInfo, ...)
{
    unsigned msgID=0;
    va_list vl;
    va_start(vl,msgInfo);

    if(msgInfo == CSV_SPECIFIC_MSG)
        msgID = va_arg(vl,int);
    else msgID = m_currentMsgID;

    for(unsigned fieldID=0;fieldID<m_types.size();fieldID++)
    {
        if(m_types[fieldID] != CSV_IGNORE)
            getField(msgID, fieldID , va_arg(vl,void*));
    }
    va_end(vl);
}

void CSVData::clearMsgs()
{
    unsigned msgID, fieldID;
    m_currentMsgID=0u;

    for(msgID = 0 ; msgID < m_values.size() ; msgID++)
    {
        for(fieldID = 0 ; fieldID < m_types.size(); fieldID++)
        {
            // Interpret and delete field
            switch(m_types[fieldID])
            {
                case CSV_STRING:
                    delete (std::string*) m_values[msgID][fieldID];
                break;
                case CSV_INT:
                    delete (int*) m_values[msgID][fieldID];
                break;
                case CSV_LLU:
                    delete (long long unsigned int*) m_values[msgID][fieldID];
                break;
                case CSV_FLOAT:
                    delete (float*) m_values[msgID][fieldID];
                break;
                case CSV_DOUBLE:
                    delete (double*) m_values[msgID][fieldID];
                break;
                case CSV_INVALID:
                default:
                    // Nothing to do
                break;
            }
        }
    }
    m_values.clear();
}

/**
 * @brief
 *  If is not last menssage, go to next menssage.
 *
 * @return bool - true If we went to next menssage,
 *  false otherwise.
 */
bool CSVData::nextMsg()
{
    if(m_currentMsgID < m_values.size())
    {
        m_currentMsgID++;
        return true;
    }
    return false;
}

void CSVData::toBegin()
{
    m_currentMsgID = 0u;
}
