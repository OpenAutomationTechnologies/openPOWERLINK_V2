/*
 * File:		assert.c
 * Purpose:		Provide macro for software assertions
 *
 * Notes:		ASSERT macro defined in assert.h calls assert_failed()
 */

#include "common.h"

/********************************************************************/
void
assert_failed(char *file, int line)
{ 
#if 0
	const char ASSERT_FAILED_STR[] = "Assertion failed in %s at line %d\n";
	printf(ASSERT_FAILED_STR, file, line);
#else
	const int led_delay = 0x100000;
	(void) file;
	(void) line;

	while (1)
	{
	}
#endif
}
/********************************************************************/
