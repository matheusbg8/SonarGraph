#include <CSVReader/CSVReader.h>
#include <CSVReader/CSVData.h>
#include <CSVReader/DataExtractorUtilities.h>
#include <cstring>
#include <string>

unsigned CSVReader::_fieldCount(FILE *file, const char spliter)
{
    if(file == 0x0) return 0u;

    fpos_t pos;
    fgetpos(file , &pos);

    char msg[2000], *pmsg, field[1000];
    pmsg = msg;

    fgets(msg,2000,file);
    int fieldCount;
    for(fieldCount = 0;
        sExtractField(&pmsg,field,spliter) != -1 ;
        fieldCount++);

    fsetpos(file, &pos);
    return fieldCount;
}

bool CSVReader::_isCommentLine(FILE *file)
{
    if(file == 0x0)
        return false;

    fpos_t pos;
    fgetpos(file , &pos);

    char c;
    while( (c = fgetc(file)) == ' ' && c != '\n' && c != '\0' && c != -1);

    fsetpos(file, &pos);

    return c == '#';
}

CSVData* CSVReader::read(const char* fileName, const char spliter, ...)
{
    using namespace std;

    // Open CSV File
    FILE *file = 0x0;
    file = fopen(fileName,"r");

    if(file == 0x0)
    {
        printf("File %s not such\n", fileName);
        return 0x0;
    }

    // Read control
    bool fileHasMsg = true;
    unsigned msgID, fieldID;
    char field[10000];

    // Value Types
    double doubleValue;
    float floatValue;
    int intValue;
    long long unsigned int lluValue;

    while(_isCommentLine(file))
        fgets(field,10000,file);

    // Count CSV fileds
    unsigned fieldCount = _fieldCount(file,spliter);
    printf("%u fields!\n", fieldCount);

    // Create a CSVData
    CSVData *data = new CSVData;
    data->m_types.resize(fieldCount,CSV_INVALID);

    // Read field type definition
    va_list vl;
    va_start(vl,spliter);
    for(unsigned i=0;i<fieldCount;i++)
    {
        data->m_types[i] = (CSV_TYPE) va_arg(vl,int);
    }
    va_end(vl);


    //Read CSV File
    for(msgID = 0; fileHasMsg ; msgID++)
    {
        // Read next menssage

        // Read next field value
        for(fieldID = 0; fieldID < fieldCount &&
            (fileHasMsg = extractField(file,field,spliter) != -1);
            fieldID++)
        {
            if(fieldID == 0)
                data->m_values.push_back(vector<void*>(fieldCount, 0x0));

            // Interpret field
            switch(data->m_types[fieldID])
            {
                case CSV_STRING:
                    data->m_values[msgID][fieldID] = new string(field);
                break;
                case CSV_INT:
                    sscanf(field,"%d", &intValue);
                    data->m_values[msgID][fieldID] = new int(intValue);
                break;
                case CSV_LLU:
                    sscanf(field,"%llu", &lluValue);
                    data->m_values[msgID][fieldID] = new long long unsigned int(lluValue);
                break;
                case CSV_FLOAT:
                    sscanf(field,"%f", &floatValue);
                    data->m_values[msgID][fieldID] = new float(lluValue);
                break;
                case CSV_DOUBLE:
                    sscanf(field,"%lf", &doubleValue);
                    data->m_values[msgID][fieldID] = new double(doubleValue);
                break;
                case CSV_INVALID:
                default:
                    // Nothing to do
                break;
            }
        }
    }

    // Close CSV file
    fclose(file);

    return data;
}

