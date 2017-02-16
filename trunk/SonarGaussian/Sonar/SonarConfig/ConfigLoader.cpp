#include "Sonar/SonarConfig/ConfigLoader.h"

#include "CSVReader/DataExtractorUtilities.h"
#include "Sonar/SonarConfig/Automaton.h"
#include <cstdio>

const string ConfigLoader::empty("");

ConfigLoader::ConfigLoader(const char *fileName):
    extractTag(5), extractValue(8)
{
    extractTag.setInitialState(0);
    extractTag.addFinalState(3);

    extractTag.add(AutomatonEdge(0,1,"[",false,false));
    extractTag.add(AutomatonEdge(0,4,"#",false,false));

    extractTag.add(AutomatonEdge(1,2," ]#",true,true));
    extractTag.add(AutomatonEdge(1,0,"]",false,false));
    extractTag.add(AutomatonEdge(1,4,"#",false,false));

    extractTag.add(AutomatonEdge(2,2," ]#",true,true));
    extractTag.add(AutomatonEdge(2,4,"#",false,false));
    extractTag.add(AutomatonEdge(2,3,"]",false,false));

    extractTag.add(AutomatonEdge(4,0,"\n",false,false));


    extractValue.setInitialState(0);
    extractValue.addFinalState(7);

    extractValue.add(AutomatonEdge(0,1," =\n#",true,true));
    extractValue.add(AutomatonEdge(0,4,"#",false,false));

    // State 1 save value name
    extractValue.add(AutomatonEdge(1,1," =\n#",true,true));
    extractValue.add(AutomatonEdge(1,4,"#",false,false));
    extractValue.add(AutomatonEdge(1,2,"=",false,false));

    extractValue.add(AutomatonEdge(2,3," \n#",true,true));
    extractValue.add(AutomatonEdge(2,5,"#",false,false));

    // State 3 save the value
    extractValue.add(AutomatonEdge(3,3,"\n#",true,true));
    extractValue.add(AutomatonEdge(3,7,"\n",false,false));
    extractValue.add(AutomatonEdge(3,6,"#",false,false));

    extractValue.add(AutomatonEdge(4,0,"\n",false,false));

    extractValue.add(AutomatonEdge(5,2,"\n",false,false));

    extractValue.add(AutomatonEdge(6,7,"\n",false,false));

    if(fileName != 0x0)
        load(fileName);
    else
        cout << "Config Loader: Empty config file name!" << endl;
}

bool ConfigLoader::load(const char *fileName)
{
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
    {
        printf("Config file %s can't be opended!\n", fileName);
        return false;
    }
//    cout << "Processing configs" << endl;
    map<string,string> *values=0x0;
    char c;
    while(c = std::getc(f))
    {
        if(c == -1) break;
        if(c == '\r') continue;
//        cout << c;

        extractTag.processSymbol(c);
        extractValue.processSymbol(c);

        if(extractTag.isFinish())
        {
            values = &(configs[extractTag.stateRecords(2)]);
            extractTag.reset();
            extractValue.reset();
        }
        if(extractValue.isFinish() && values != 0x0)
        {
            (*values)[extractValue.stateRecords(1)] = extractValue.stateRecords(3);
            extractTag.reset();
            extractValue.reset();
        }
    }
    fclose(f);
}

bool ConfigLoader::has(const char *tag, const char *key)
{
    map<string, map<string,string> >::iterator ic = configs.find(tag);

    if(ic == configs.end())
        return false;

    map<string,string> &m = ic->second;
    map<string,string>::iterator im = m.find(key);

    if(im == m.end())
        return false;

    return true;
}

const string &ConfigLoader::getString(const char *tag, const char *key, bool *ok)
{
    map<string, map<string,string> >::iterator ic = configs.find(tag);

    if(ic == configs.end())
    {
        if(ok != 0x0) *ok = false;
        else
            cout << "Config tag " << tag << " not found!" << endl;

        return empty;
    }

    map<string,string> &m = ic->second;
    map<string,string>::iterator im = m.find(key);

    if(im == m.end())
    {
        if(ok != 0x0) *ok = false;
        else
            cout << "Confg key "
                 << tag << "->" << key
                 << " was not found!" << endl;

        return empty;
    }

    if(ok != 0x0) *ok = true;
    return im->second;
}

bool ConfigLoader::getString(const char *tag, const char *key, string *str)
{
    map<string, map<string,string> >::iterator ic = configs.find(tag);

    if(ic == configs.end())
    {
        return false;
    }

    map<string,string> &m = ic->second;
    map<string,string>::iterator im = m.find(key);

    if(im == m.end())
    {
        return false;
    }

    *str = im->second;
    return true;
}

float ConfigLoader::getFloat(const char *tag, const char *key)
{
    float value=0.f;
    sscanf(getString(tag,key).c_str(), "%f",&value);
    return value;
}

bool ConfigLoader::getFloat(const char *tag, const char *key, float *value)
{
    bool ok;
    sscanf(getString(tag,key,&ok).c_str(), "%f",value);
    return ok;
}

int ConfigLoader::getInt(const char *tag, const char *key)
{
    int value=0;
    sscanf(getString(tag,key).c_str(), "%d",&value);
    return value;
}

bool ConfigLoader::getInt(const char *tag, const char *key, int *value)
{
    bool ok;
    sscanf(getString(tag,key,&ok).c_str(), "%d",value);
    return ok;
}

void ConfigLoader::printConfigs()
{
    map<string,map<string,string> >::iterator imm;

    for(imm = configs.begin(); imm != configs.end(); imm++)
    {
        cout << "TAG " << imm->first << endl;
        map<string,string> &mp = imm->second;
        map<string,string>::iterator im;
        for(im = mp.begin(); im != mp.end() ; im++)
        {
            cout << "VALUE "
                 << im->first
                 << " = "
                 << im->second
                 << endl;
        }
    }
}
