/*
 * File:		assert.c
 * Purpose:		Provide macro for software assertions
 *
 * Notes:		ASSERT macro defined in assert.h calls assert_failed()
 */

#include "../include/common.h"

/********************************************************************/
void
assert_failed(char *file, int line)
{ 
	const char ASSERT_FAILED_STR[] = "Assertion failed in %s at line %d\n";
	printf(ASSERT_FAILED_STR, file, line);
}
/********************************************************************/
