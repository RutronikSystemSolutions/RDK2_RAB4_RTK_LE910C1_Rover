/*
 * strutils.h
 *
 *  Created on: 13 Aug 2024
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#ifndef STRUTILS_H_
#define STRUTILS_H_

int strutils_count_char(char* string, int string_len, char c);

int strutils_split_and_extract(char* string, int string_len, char separator, int index, char* output_string);

#endif /* STRUTILS_H_ */
