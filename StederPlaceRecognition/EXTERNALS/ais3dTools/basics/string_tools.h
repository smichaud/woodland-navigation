#ifndef AIS3DTOOLS_STRING_TOOLS_H
#define AIS3DTOOLS_STRING_TOOLS_H

#include <string>
#include <sstream>
#include <cstdlib>
#include <vector>
#include "runtime_error.h"


namespace Ais3dTools {

/** @addtogroup utils **/
// @{

/** \file stringTools.h
 * \brief utility functions for handling strings
 */

/**
 * remove whitespaces from the start/end of a string
 */
std::string trim(const std::string& s);

/**
 * remove whitespaces from the left side of the string
 */
std::string trimLeft(const std::string& s);

/**
 * remove whitespaced from the right side of the string
 */
std::string trimRight(const std::string& s);

/**
 * convert the string to lower case
 */
std::string strToLower(const std::string& s);

/**
 * convert a string to upper case
 */
std::string strToUpper(const std::string& s);

/**
 * read integer values (seperated by spaces) from a string and store
 * them in the given OutputIterator.
 */
template <typename OutputIterator>
OutputIterator readInts(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    int id = strtol(cl, &cle, 10);
    if (cl == cle)
      break;
    *out++ = id;
    cl = cle;
  }
  return out;
}

/**
 * read float values (seperated by spaces) from a string and store
 * them in the given OutputIterator.
 */
template <typename OutputIterator>
OutputIterator readFloats(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    double val = strtod(cl, &cle);
    if (cl == cle)
      break;
    *out++ = val;
    cl = cle;
  }
  return out;
}

/**
 * format a string and return a std::string.
 * Format is just like printf, see man 3 printf
 */
std::string formatString(const char* fmt, ...); // __attribute__ ((format (printf, 1, 2)));

/**
 * replacement function for sprintf which fills a std::string instead of a char*
 */
int strPrintf(std::string& str, const char* fmt, ...);// __attribute__ ((format (printf, 2, 3)));

/**
 * convert a string into an other type.
 */
template<typename T>
void convertString(const std::string& s, T& x, bool failIfLeftoverChars = true)
{
  std::istringstream i(s);
  char c;
  if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
    throw RuntimeError("%s: Unable to convert %s", __PRETTY_FUNCTION__, s.c_str());
}

/**
 * convert a string into an other type.
 * Return the converted value. Throw error if parsing is wrong.
 */
template<typename T>
T stringToType(const std::string& s, bool failIfLeftoverChars = true)
{
  T x;
  convertString(s, x, failIfLeftoverChars);
  return x;
}

/**
 * return true, if str starts with substr
 */
bool strStartsWith(const std::string & str, const std::string& substr);

/**
 * return true, if str ends with substr
 */
bool strEndsWith(const std::string & str, const std::string& substr);

/**
 * expand the given filename like a posix shell, e.g., ~ $CARMEN_HOME and other will get expanded.
 * Also command substitution, e.g. `pwd` will give the current directory.
 */
std::string strExpandFilename(const std::string& filename);

/**
 * split a string into token based on the characters given in delim
 */
std::vector<std::string> strSplit(const std::string& s, const std::string& delim);

/**
 * read a line from is into currentLine.
 * @return the number of characters read into currentLine (excluding newline), -1 on eof()
 */
int readLine(std::istream& is, std::stringstream& currentLine);

// @}

} //end namespace
#endif
