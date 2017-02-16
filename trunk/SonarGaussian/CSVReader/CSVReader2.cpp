#include <CSVReader/CSVReader2.h>
#include <CSVReader/CSVData.h>
#include <CSVReader/DataExtractorUtilities.h>
#include <cstring>
#include <string>

CSVReader2::CSVReader2():
    csvFile(0x0),spliter('\0')
{

}

CSVReader2::~CSVReader2()
{
    close();
}

bool CSVReader2::open(const char *fileName, unsigned nCols, const char spliter, ...)
{
    // Open FILE
    csvFile = fopen(fileName,"r");
    if(csvFile == 0x0) return false;
    csvFileName = fileName;

    this->spliter = spliter;

    // Read field types definition
    types.reserve(nCols);
    va_list vl;
    va_start(vl,spliter);
    for(unsigned i=0;i<nCols;i++)
    {
        CSV_TYPE t = (CSV_TYPE) va_arg(vl,int); // Passing int because warnnings
        if(t != CSV_IGNORE)
            types.push_back(t);
    }
    va_end(vl);

    // Clean pointer adresses
    adress.clear();
    adress.resize(types.size(),0x0);

    return true;
}

void CSVReader2::close()
{
    if(csvFile!= 0x0)
    {
        fclose(csvFile);
        csvFile = 0x0;
        types.clear();
        adress.clear();
        csvFileName.clear();
        spliter = '\0';
    }
}

void CSVReader2::ignoreLine()
{
    readLine(csvFile);
}

bool CSVReader2::read(void *field1,...)
{
    if(csvFile == 0x0) return false;

    adress[0] = field1;

    // Get data adress
    va_list vl;
    va_start(vl,field1);
    for(unsigned i=1;i<adress.size();i++)
    {
        adress[i] = va_arg(vl,void*);
    }
    va_end(vl);

    // Read control
    char field[1000];
    bool hasMsg = false;

    // Read CSV line
    for(unsigned fieldID = 0;
        fieldID < adress.size() &&
       (hasMsg = extractField(csvFile,field,spliter) != -1);
        fieldID++)
    {
        // Interpret field
        switch(types[fieldID])
        {
            case CSV_STRING:
                *((string*) adress[fieldID]) = field;
            break;
            case CSV_INT:
                sscanf(field,"%d", (int*) adress[fieldID]);
            break;
            case CSV_LLU:
                sscanf(field,"%llu", (long long unsigned int*) adress[fieldID]);
            break;
            case CSV_FLOAT:
                sscanf(field,"%f", (float*) adress[fieldID]);
            break;
            case CSV_DOUBLE:
                sscanf(field,"%lf", (double*) adress[fieldID]);
            break;
            case CSV_INVALID:
            default:
                // Nothing to do
            break;
        }
    }
    return hasMsg;
}
