/*********************************************************************
 *
 * Copyright:
 *	FREESCALE, INC. All Rights Reserved.  
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of FREESCALE, Inc. This 
 *  software is provided on an "AS IS" basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, FREESCALE 
 *  DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED, INCLUDING 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
 *  PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH REGARD TO THE 
 *  SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF) AND ANY 
 *  ACCOMPANYING WRITTEN MATERIALS.
 * 
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL FREESCALE BE LIABLE FOR ANY DAMAGES WHATSOEVER (INCLUDING 
 *  WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS 
 *  INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY
 *  LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.   
 * 
 *  FREESCALE assumes no responsibility for the maintenance and support
 *  of this software
 ********************************************************************/

/*
 * File:		stdlib.c
 * Purpose:		Standard functions implementation.
 */

#include "plccore.h"

/****************************************************************/
int
isspace (int ch)
{
	if ((ch == ' ') || (ch == '\t'))	
		return TRUE;
	else
		return FALSE;
}

/****************************************************************/
int
isalnum (int ch)
{
	/* ASCII only */
	if (((ch >= '0') && (ch <= '9')) ||
		((ch >= 'A') && (ch <= 'Z')) ||
		((ch >= 'a') && (ch <= 'z')))
		return TRUE;
	else
		return FALSE;
}

/****************************************************************/
int
isdigit (int ch)
{
	/* ASCII only */
	if ((ch >= '0') && (ch <= '9'))
		return TRUE;
	else
		return FALSE;
}

/****************************************************************/
int
isupper (int ch)
{
	/* ASCII only */
	if ((ch >= 'A') && (ch <= 'Z'))
		return TRUE;
	else
		return FALSE;
}

/****************************************************************/
int
strcasecmp (const char *s1, const char *s2)
{
	char	c1, c2;
	int		result = 0;

	while (result == 0)
	{
		c1 = *s1++;
		c2 = *s2++;
		if ((c1 >= 'a') && (c1 <= 'z'))
			c1 = (char)(c1 - ' ');
		if ((c2 >= 'a') && (c2 <= 'z'))
			c2 = (char)(c2 - ' ');
		if ((result = (c1 - c2)))
			break;
		if ((c1 == 0) || (c2 == 0))
			break;
	}
	return result;
}


/****************************************************************/
int
strncasecmp (const char *s1, const char *s2, int n)
{
	char	c1, c2;
	int		k = 0;
	int		result = 0;

	while ( k++ < n )
	{
		c1 = *s1++;
		c2 = *s2++;
		if ((c1 >= 'a') && (c1 <= 'z'))
			c1 = (char)(c1 - ' ');
		if ((c2 >= 'a') && (c2 <= 'z'))
			c2 = (char)(c2 - ' ');
		if ((result = (c1 - c2)))
			break;
		if ((c1 == 0) || (c2 == 0))
			break;
	}
	return result;
}

/****************************************************************/
uint32
strtoul (char *str, char **ptr, int base)
{
	unsigned long rvalue;
	int c, err, neg;
	char *endp;
	char *startp;

	rvalue = 0;  err = 0;  neg = 0;

	/* Check for invalid arguments */
	if ((str == NULL) || (base < 0) || (base == 1) || (base > 36))
	{
		if (ptr != NULL)
		{
			*ptr = str;
		}
		return 0;
	}

	/* Skip leading white spaces */
	for (startp = str; isspace(*startp); ++startp)
		;

	/* Check for notations */
	switch (startp[0])
	{
		case '0':
			if ((startp[1] == 'x') || (startp[1] == 'X'))
			{
				if ((base == 0) || (base == 16))
				{
					base = 16;
					startp = &startp[2];
				}
			}
			break;
		case '-':
			neg = 1;
			startp = &startp[1];
			break;
		default:
			break;
	}

	if (base == 0)
		base = 10;

	/* Check for invalid chars in str */
	for ( endp = startp; (c = *endp) != '\0'; ++endp)
	{
		/* Check for 0..9,Aa-Zz */
		if (!isalnum(c))
		{
			err = 1;
			break;
		}

		/* Convert char to num in 0..36 */
		if (isdigit(c))
		{
			c = c - '0';
		}
		else
		{
			if (isupper(c))
			{
				c = c - 'A' + 10;
			}
			else
			{
				c = c - 'a' + 10;
			}
		}

		/* check c against base */
		if (c >= base)
		{
			err = 1;
			break;
		}

		if (neg)
		{
			rvalue = (rvalue * base) - c;
		}
		else
		{
			rvalue = (rvalue * base) + c;
		}
	}

	/* Upon exit, endp points to the character at which valid info */
	/* STOPS.  No chars including and beyond endp are used.        */

	if (ptr != NULL)
		*ptr = endp;

	if (err)
	{
		if (ptr != NULL)
			*ptr = str;
		
		return 0;
	}
	else
	{
		return rvalue;
	}
}

/****************************************************************/
int
strlen (const char *str)
{
	char *s = (char *)str;
	int len = 0;

	if (s == NULL)
		return 0;

	while (*s++ != '\0')
		++len;

	return len;
}

/****************************************************************/
char *
strcat (char *dest, const char *src)
{
	char *dp;
	char *sp = (char *)src;

	if ((dest != NULL) && (src != NULL))
	{
		dp = &dest[strlen(dest)];

		while (*sp != '\0')
		{
			*dp++ = *sp++;
		}
		*dp = '\0';
	}
	return dest;
}

/****************************************************************/
char *
strncat (char *dest, const char *src, int n)
{
	char *dp;
	char *sp = (char *)src;

	if ((dest != NULL) && (src != NULL) && (n > 0))
	{
		dp = &dest[strlen(dest)];

		while ((*sp != '\0') && (n-- > 0))
		{
			*dp++ = *sp++;
		}
		*dp = '\0';
	}
	return dest;
}

/****************************************************************/
char *
strcpy (char *dest, const char *src)
{
	char *dp = (char *)dest;
	char *sp = (char *)src;

	if ((dest != NULL) && (src != NULL))
	{
		while (*sp != '\0')
		{
			*dp++ = *sp++;
		}
		*dp = '\0';
	}
	return dest;
}

/****************************************************************/
char *
strncpy (char *dest, const char *src, int n)
{
	char *dp = (char *)dest;
	char *sp = (char *)src;

	if ((dest != NULL) && (src != NULL) && (n > 0))
	{
		while ((*sp != '\0') && (n-- > 0))
		{
			*dp++ = *sp++;
		}
		*dp = '\0';
	}
	return dest;
}

/****************************************************************/
int
strcmp (const char *s1, const char *s2)
{
	/* No checks for NULL */
	char *s1p = (char *)s1;
	char *s2p = (char *)s2;

	while (*s2p != '\0')
	{
		if (*s1p != *s2p)
			break;

		++s1p;
		++s2p;
	}
	return (*s1p - *s2p);
}

/****************************************************************/
int
strncmp (const char *s1, const char *s2, int n)
{
	/* No checks for NULL */
	char *s1p = (char *)s1;
	char *s2p = (char *)s2;

	if (n <= 0)
		return 0;

	while (*s2p != '\0')
	{
		if (*s1p != *s2p)
			break;

		if (--n == 0)
			break;

		++s1p;
		++s2p;
	}
	return (*s1p - *s2p);
}


//----------------------------------------------------------------
//  2006/06/19 -rs
//----------------------------------------------------------------
char *
strstr (const char * str1, const char * str2)
{

char* cp = (char*)str1;
char* s1, *s2;


    if ( !*str2 )
    {
        return ((char*)str1);
    }

    while (*cp)
    {
        s1 = cp;
        s2 = (char*) str2;

        while ( *s1 && *s2 && !(*s1-*s2) )
        {
            s1++, s2++;
        }

        if (!*s2)
        {
            return (cp);
        }

        cp++;
    }

    return (NULL);

}
//----------------------------------------------------------------

/****************************************************************/
/*
void *
memcpy (void *dest, const void *src, unsigned n)
{
	int longs, bytes;
	uint32 *dpl = (uint32 *)dest;
	uint32 *spl = (uint32 *)src;
	uint8  *dpb, *spb;

	if ((dest != NULL) && (src != NULL) && (n > 0))
	{
		bytes = (n & 0x3);
		longs = (n - bytes) >> 2;
	
		while (longs--)
			*dpl++ = *spl++;
		
		dpb = (uint8 *)dpl;
		spb = (uint8 *)spl;
		
		while (bytes--)
			*dpb++ = *spb++;
	}
	return dest;
}
*/
/*
asm void memcpy (void *dest, void *src, uint32 n)
{
        tst.l    4(a7)
        beq.s    LOOP_OUT2
        
        tst.l    8(a7)
        beq.s    LOOP_OUT2
        
        move.l   12(a7),d0 //n
        
        tst.l    d0
        beq.s    LOOP_OUT2
        
        movea.l  4(a7),a1  //dest
        movea.l  8(a7),a0  //src
           
        //==== 4 byte transfer =====
        move.l   d0,d1
        lsr.l    #2,d1
        beq.s LOOP_OUT1
 LOOP_BEGIN1:
        move.l   (a0)+,(a1)+
        subq.l   #1,d1
        bne.s    LOOP_BEGIN1
 LOOP_OUT1:          
        //==== 1 byte transfer =====
        and.l    #3,d0
        beq.s LOOP_OUT2
 LOOP_BEGIN2:
        move.b   (a0)+,(a1)+
        subq.l   #1,d0
        bne.s    LOOP_BEGIN2
 LOOP_OUT2:
        rts

}
*/

/****************************************************************/
#if 0
void *
memset (void *s, int c, unsigned n)
{
	/* Not optimized, but very portable */
	unsigned char *sp = (unsigned char *)s;

	if ((s != NULL) && (n > 0))
	{
		while (n--)
		{
			*sp++ = (unsigned char)c;
		}
	}
	return s;
}
#endif
/************************************************************************
* NAME: memcmp
*
* DESCRIPTION: Compare two memory regions and return zero if they are identical,
*              non-zero otherwise.  If count is zero, return zero.
*************************************************************************/

int memcmp (char * from, char * to, int count)
{
  int rtnval = 0;

  while (count-- > 0)
    {
      if (*from++ != *to++)
	{
	  rtnval = 1;
	  break;
	}
    }
  return (rtnval);
}
