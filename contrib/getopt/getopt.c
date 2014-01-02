/**
********************************************************************************
\file   getopt.c

\brief  get command line options

The file implements the getopt() function used to parse command line options.

\ingroup module_lib_getopt
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 1987, 1993, 1994
      The Regents of the University of California.  All rights reserved.
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define BADCH   (int)'?'
#define BADARG  (int)':'
#define EMSG    ""

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
int     opterr = 1;         /* if error message should be printed */
int     optind = 1;         /* index into parent argv vector */
int     optopt;             /* character checked for validity */
int     optreset;           /* reset getopt */
char*   optarg;             /* argument associated with option */

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Get command line option

The funtion implements a command line parser compatible with Linux standard
getopt() functon.


\param  nargc           Number of arguments.
\param  nargv           Pointer to argument string.
\param  ostr            pointer to option string.

\return If an option was successfully found, then getopt() returns the option
        character. If all command-line options have been parsed, then getopt()
        returns -1. If getopt() encounters an option character that was not in
        optstring, then '?' is returned. If getopt() encounters an option with
        a missing argument, then the return value depends on the first character
        in optstring: if it is ':', then ':' is returned; otherwise '?' is
        returned.
*/
//------------------------------------------------------------------------------
int getopt(int nargc, char* const nargv[], const char* ostr)
{
    static char*    place = EMSG;              /* option letter processing */
    char*           oli;                       /* option letter list index */

    if (optreset || *place == 0)
    {          /* update scanning pointer */
        optreset = 0;
        place = nargv[optind];
        if (optind >= nargc || *place++ != '-')
        {
                /* Argument is absent or is not an option */
                place = EMSG;
                return (-1);
        }

        optopt = *place++;
        if (optopt == '-' && *place == 0)
        {
                /* "--" => end of options */
                ++optind;
                place = EMSG;
                return (-1);
        }

        if (optopt == 0)
        {
            /* Solitary '-', treat as a '-' option
               if the program (eg su) is looking for it. */
            place = EMSG;
            if (strchr(ostr, '-') == NULL)
                    return (-1);
            optopt = '-';
        }
    }
    else
    {
        optopt = *place++;
    }

    /* See if option letter is one the caller wanted... */
    if (optopt == ':' || (oli = strchr(ostr, optopt)) == NULL)
    {
        if (*place == 0)
            ++optind;
        if (opterr && *ostr != ':')
            (void)fprintf(stderr, "illegal option -- %c\n",optopt);
        return (BADCH);
    }

    /* Does this option need an argument? */
    if (oli[1] != ':')
    {
        /* don't need argument */
        optarg = NULL;
        if (*place == 0)
            ++optind;
    }
    else
    {
        /* Option-argument is either the rest of this argument or the
           entire next argument. */
        if (*place)
            optarg = place;
        else if (nargc > ++optind)
            optarg = nargv[optind];
        else
        {
            /* option-argument absent */
            place = EMSG;
            if (*ostr == ':')
                return (BADARG);
            if (opterr)
                (void)fprintf(stderr, "option requires an argument -- %c\n", optopt);
            return (BADCH);
        }
        place = EMSG;
        ++optind;
    }
    return (optopt);                        /* return option letter */
}

