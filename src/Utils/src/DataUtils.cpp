/**
 * @file src/DataUtils.cpp
 *
 * This file implements the class DataUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>
#include "Utils/include/DataUtils.h"
#include "Utils/include/PrintUtils.h"

namespace DataUtils
{
  template<typename T>
  string varToString(const T& var)
  {
    stringstream ss;
    try {
      ss << var;
    } catch (exception &e) {
      LOG_EXCEPTION(
        "Exception caught in DataUtils::varToString(): " << e.what()
      );
    }
    return ss.str();
  }

  template string varToString<int>(const int&);
  template string varToString<unsigned>(const unsigned&);
  template string varToString<unsigned long>(const unsigned long&);
  template string varToString<float>(const float&);
  template string varToString<double>(const double&);
  template string varToString<string>(const string&);

  template<typename T>
  void stringToVar(const string& str, T& var)
  {
    /*try {
      istringstream istr(str);
      if (!(istr >> std::boolalpha >> var))
        throw("Invalid type for string to variable conversion");
    } catch (exception &e) {
      LOG_EXCEPTION(
        "Exception caught in DataUtils::stringToVar(): " << e.what()
      );
    }*/
    try {
      var = boost::lexical_cast<T>(str);
    } catch (const boost::bad_lexical_cast& e) {
      LOG_EXCEPTION(
        "Cannot convert string: " <<
        str <<
        " to variable of specified type." <<
        boost::diagnostic_information(e));
    }
  }
  template void stringToVar<int>(const string&, int&);
  template void stringToVar<unsigned>(const string&, unsigned&);
  template void stringToVar<float>(const string&, float&);
  template void stringToVar<double>(const string&, double&);
  template void stringToVar<string>(const string&, string&);

  template<typename T>
  void stringToVar(const string& str, vector<T>& var)
  {
    string tmp = str;
    tmp = str.substr(1, str.size() - 2);
    vector<string> values = splitString(tmp, ',');
    var.resize(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
      stringToVar<T>(values[i], var[i]);
    }
  }
  template void stringToVar<int>(const string&, vector<int>&);
  template void stringToVar<unsigned>(const string&, vector<unsigned>&);
  template void stringToVar<float>(const string&, vector<float>&);
  template void stringToVar<double>(const string&, vector<double>&);
  template void stringToVar<string>(const string&, vector<string>&);


  string bytesToHexString(unsigned char* buffer, const int& size)
  {
    size_t bufferSize = sizeof(buffer);
    if (size != -1) bufferSize = size;
    char* converted = new char[bufferSize * 2 + 1];
    for (size_t i = 0; i < bufferSize; ++i)
      sprintf(&converted[i * 2], "%02X", buffer[i]);
    string temp(converted);
    delete[] converted;
    return temp;
  }

  string convertBytesToString(const unsigned char*& buffer, const int& size)
  {
    int bufferSize = sizeof(buffer);
    if (size != -1) bufferSize = size;
    string temp = "";
    for (int i = 0; i < bufferSize; i++) {
      if (buffer[i] == '\n') temp += '\n';
      temp += buffer[i];
    }
    return temp;
  }

  vector<string>& splitString(const string& s, const char& delim, vector<string>& elems)
  {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
      elems.push_back(item);
    }
    return elems;
  }

  vector<string> splitString(const string& s, const char& delim)
  {
    std::vector < std::string > elems;
    splitString(s, delim, elems);
    return elems;
  }

  string getCurrentDateTime()
  {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
    string str(buffer);
    return str;
  }
}
