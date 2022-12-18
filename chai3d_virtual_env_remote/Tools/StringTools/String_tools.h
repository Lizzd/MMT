//
// Created by zican on 21.09.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_STRING_TOOLS_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_STRING_TOOLS_H

#include <iostream>
#include <vector>
#include <string>
using namespace std;

class String_tools{
public:
    static int string_split(string input,vector<string> & output,string &&delimiter)
    {
        try{
            size_t pos=0;
            std::string token;
            while((pos=input.find(delimiter))!=string::npos)
            {
                //cout<<"[string split]"<<input<<endl;
                token=input.substr(0,pos);
                output.push_back(token);
                input.erase(0,pos+delimiter.length());
            }
            output.push_back(input);
        }catch(exception e){cout<<"string_split error:"<<e.what()<<endl;}
        //cout<<"split done"<<endl;
        return (int)output.size();
    }

};


#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_STRING_TOOLS_H
