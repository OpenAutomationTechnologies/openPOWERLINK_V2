
/************************************************************************
*
*  FILE NAME: stdlib.h
*
*  PURPOSE: Standard functions definitions.
*
***********************************************************************/


#ifndef _STDLIB_H
#define _STDLIB_H

/********************************************************************
* Standard library functions
*********************************************************************/

int
isspace (int);

int
isalnum (int);

int
isdigit (int);

int
isupper (int);

int
strcasecmp (const char *, const char *);

int
strncasecmp (const char *, const char *, int);

unsigned long
strtoul (char *, char **, int);

int
strlen (const char *);

char *
strcat (char *, const char *);

char *
strncat (char *, const char *, int);

char *
strcpy (char *, const char *);

char *
strncpy (char *, const char *, int);

int
strcmp (const char *, const char *);

int
strncmp (const char *, const char *, int);

char *
strstr (const char * str1, const char * str2);   //  2006/06/19 -rs

void *
memcpy (void *, const void *, unsigned);

void *
memset (void *, int, unsigned);

int memcmp(const void * src1, const void * src2, unsigned long n);

void
free (void *);
 
void *
malloc (unsigned);


#endif
