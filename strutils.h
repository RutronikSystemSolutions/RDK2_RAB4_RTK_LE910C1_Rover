/*
 * strutils.h
 *
 *  Created on: 2024.04.04
 *      Author: RJ030
 */

#ifndef STRUTILS_H_
#define STRUTILS_H_

int strutils_count_char(char* string, int string_len, char c);

int strutils_split_and_extract(char* string, int string_len, char separator, int index, char* output_string);

#endif /* STRUTILS_H_ */
