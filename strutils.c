#include "strutils.h"

#include <string.h>

int strutils_count_char(char* string, int string_len, char c)
{
    int count = 0;
    for (int i = 0; i < string_len; ++i)
    {
        if (string[i] == c) count++;
    }
    return count;
}

int strutils_split_and_extract(char* string, int string_len, char separator, int index, char* output_string)
{
    int current_index = 0;
    int last_separator_index = 0;

    for(int i = 0; i < string_len; ++i)
    {
        if(string[i] == separator)
        {
            if (current_index == index)
            {
                int len = i - last_separator_index - 1;
                // Good
                memcpy(output_string, &string[last_separator_index+1], len);
                return len;
            }
            current_index++;
            last_separator_index = i;
        }
    }

    // End of the string
    if (current_index == index)
    {
        // Extract the last part
        int len = string_len - last_separator_index;
        memcpy(output_string, &string[last_separator_index+1], len);
        return len;
    }

    return -1;
}
