#include "cvg_string_conversions.h"

std::string cvg_int_to_string(int i) {
    static char buffer[16];
    snprintf ( buffer, 16, "%d", i);
    // itoa (i,buffer,16);
    std::string result_str(buffer);
    return result_str;
}

int    cvg_string_to_int(std::string str) {
    return strtol(str.c_str(),NULL,10);
}

double cvg_string_to_double(std::string str) {
    return strtod(str.c_str(),NULL);
}

const std::vector<std::string> cvg_convert_input_string_to_string_stdvector(const std::string &value_location)
{
    std::vector<std::string> result_str_vector;
    if (value_location.size() > XMLFILEREADER_MAXIMUM_STRING_SIZE-1) {
        throw std::runtime_error( (std::string("convert_input_string_to_string_stdvector") + " configuration string is too long") );
    }
    char str[XMLFILEREADER_MAXIMUM_STRING_SIZE];
    strcpy(str, value_location.c_str());
    char * pch; char *saveptr;
    // printf ("Splitting string \"%s\" into tokens:\n",str);
    pch = strtok_r( str, ":", &saveptr);
    while (pch != NULL)
    {
//      std::cout << pch << std::endl;
      result_str_vector.push_back(pch);
      pch = strtok_r( NULL,":", &saveptr);
    }
    return result_str_vector;
}
