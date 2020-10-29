#include <iostream>
#include <fstream>
#include <map>


class ConfigParser
{
public:
  std::map<std::string, std::string> data;

  ConfigParser(std::string conf_file="./config.txt")
  {
    std::ifstream fin(conf_file.c_str());
    if(!fin)
    {
      std::cerr << "config file doesn't exist, please check it!" << std::endl;
      return;
    }

    while(!fin.eof())
    {
      std::string str;
      getline(fin, str);
      
      // for comment
      if (str[0] == '#')
        continue;

      int pos = str.find("=");
      // not a valid line
      if (pos == -1)
        continue;

      std::string key = str.substr(0, pos);
      std::string value = str.substr(pos+1, str.length());
      data[key] = value;

      if(!fin.good())
        break;
    }
    // std::cout << data.size() << std::endl;
  }

  std::string getData(std::string key)
  {
    std::map<std::string, std::string>::iterator iter = data.find(key);
    if(iter==data.end())
    {
      std::cerr<<"Parameter name "<< key << " not found!"<<std::endl;
      return std::string("NOT_FOUND");
    }
    return iter->second;
  }
};