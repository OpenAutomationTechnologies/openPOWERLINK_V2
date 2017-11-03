/**
********************************************************************************
\file   linux/ftracedebug.c

\brief  Linux ftrace debug functions

The file implements debug function using the Linux ftrace utility.

\ingroup module_debug
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include <common/oplkinc.h>             // Includes the configuration file that sets the guarding macro
#include <common/ftracedebug.h>

#if defined(FTRACE_DEBUG)
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define MAX_PATH    256
#define _STR(x)     #x
#define STR(x)      _STR(x)

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//          P R I V A T E   D E F I N I T I O N S                             //
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
static int  ftraceEnableFd_l = 0;
static int  ftraceMarkerFd_l = -1;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static const char* findDebugfs(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Open ftrace files

The function opens the tracing_on and trace_marker files.

\return The function returns an error code.
\retval 0                           Success
\retval 1                           Error

\ingroup module_debug
*/
//------------------------------------------------------------------------------
int ftrace_open(void)
{
    char        sPath[MAX_PATH];
    const char* p_debugfs;

    p_debugfs = findDebugfs();
    if (p_debugfs != NULL)
    {
        strcpy(sPath, p_debugfs);
        strcat(sPath, "/tracing/tracing_on");

        ftraceEnableFd_l = open(sPath, O_WRONLY);
        if (ftraceEnableFd_l < 0)
            return -1;
        else
        {
            strcpy(sPath, p_debugfs);
            strcat(sPath, "/tracing/trace_marker");

            ftraceMarkerFd_l = open(sPath, O_WRONLY);
            if (ftraceMarkerFd_l < 0)
            {
                close(ftraceEnableFd_l);
                return -1;
            }
        }
    }
    else
        return -1;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Close ftrace files

The function closes the tracing_on and trace_marker files.

\return The function returns always 0.

\ingroup module_debug
*/
//------------------------------------------------------------------------------
int ftrace_close(void)
{
    close(ftraceEnableFd_l);
    close(ftraceMarkerFd_l);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Enable ftrace tracing

The function enables or disables ftrace tracing by writing to the tracing_on
file.

\param[in]      fEnable_p           Enable tracing: 1 for enable and 0 for disable

\ingroup module_debug
*/
//------------------------------------------------------------------------------
void ftrace_enable(int fEnable_p)
{
    if (fEnable_p)
        write(ftraceEnableFd_l, "1", 1);
    else
        write(ftraceEnableFd_l, "0", 1);
}

//------------------------------------------------------------------------------
/**
\brief  Write a ftrace marker

The function writes a ftrace marker.

\param[in]      fmt                 Format string.
\param[in]      ...                 Arguments for the format string

\ingroup module_debug
*/
//------------------------------------------------------------------------------
void ftrace_writeTraceMarker(const char* fmt, ...)
{
    va_list argp;
    char    message[128];
    int     len;

    len = sprintf(message, "==== ");

    va_start(argp, fmt);
    len += vsprintf(message + len, fmt, argp);
    va_end(argp);

    sprintf(message + len, " ====\n");
    write(ftraceMarkerFd_l, message, strlen(message) + 1);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Find debug filesystem

The function searches the path to the debug file system on a Linux machine

\return The function returns the path to the debug file system.
*/
//------------------------------------------------------------------------------
static const char* findDebugfs(void)
{
    static char debugfs[MAX_PATH + 1];
    static int  debugfs_found;
    char        type[100];
    FILE*       fp;

    if (debugfs_found)
        return debugfs;

    fp = fopen("/proc/mounts", "r");
    if (fp == NULL)
        return NULL;

    while (fscanf(fp,
                  "%*s %" STR(MAX_PATH) "s %99s %*s %*d %*d\n",
                  debugfs,
                  type) == 2)
    {
        if (strcmp(type, "debugfs") == 0)
            break;
    }

    fclose(fp);

    if (strcmp(type, "debugfs") != 0)
        return NULL;

    debugfs_found = 1;

    return debugfs;
}

/// \}

#endif
