#ifndef CVG_STRING_CONVERSIONS_H
#define CVG_STRING_CONVERSIONS_H

#include <string>
#include <stdio.h>
#include <stdlib.h>     /* strtol */
#include <vector>
#include <stdexcept>
#include <string.h>

#define XMLFILEREADER_MAXIMUM_STRING_SIZE (1024)

std::string cvg_int_to_string(int i);

int    cvg_string_to_int(std::string str);
double cvg_string_to_double(std::string str);

const std::vector<std::string> cvg_convert_input_string_to_string_stdvector(const std::string &value_location);

#endif // CVG_STRING_CONVERSIONS_H
