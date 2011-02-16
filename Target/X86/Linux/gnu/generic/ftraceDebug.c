/*******************************************************************************

  File:         ftraceDebug.c

  (c) Bernecker + Rainer Industrie Elektronik Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Author:       Josef Baumgartner

  Description:  Linux ftrace debug functions

                ftraceDebug.c contains function for using the Linux ftrace
                tracing utility.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of copyright holders nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

*******************************************************************************/

//=========================================================================//
// Includes                                                                //
//=========================================================================//
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include "EplInc.h"

#ifdef  FTRACE_DEBUG

//=========================================================================//
// Definitions                                                             //
//=========================================================================//
#define MAX_PATH 256
#define _STR(x) #x
#define STR(x) _STR(x)


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
int iFtraceEnableFd = 0;
int iFtraceMarkerFd = -1;



//=========================================================================//
//          P R I V A T E   F U N C T I O N S                              //
//=========================================================================//

//------------------------------------------------------------------------------
// Function:    findDebugfs
//
// Description: findDebugfs() searches the path to the debug file system on a
//              Linux machine.
//
// Parameters:  N/A
//
// Return:      Path to debug filesystem
//------------------------------------------------------------------------------
static char *findDebugfs(void)
{
    static char debugfs[MAX_PATH + 1];
    static int debugfs_found;
    char type[100];
    FILE *fp;

    if (debugfs_found)
            return debugfs;

    if ((fp = fopen("/proc/mounts","r")) == NULL)
            return NULL;

    while (fscanf(fp, "%*s %"
                  STR(MAX_PATH)
                  "s %99s %*s %*d %*d\n",
                  debugfs, type) == 2) {
            if (strcmp(type, "debugfs") == 0)
                    break;
    }
    fclose(fp);

    if (strcmp(type, "debugfs") != 0)
            return NULL;

    debugfs_found = 1;

    return debugfs;
}

//=========================================================================//
//          P U B L I C   F U N C T I O N S                                //
//=========================================================================//

//------------------------------------------------------------------------------
// Function:    FtraceOpen
//
// Description: FtraceOpen() opens the tracing_on and trace_marker files.
//
// Parameters:  N/A
//
// Return:      0 on success, -1 on error
//------------------------------------------------------------------------------
int FtraceOpen(void)
{
    char        sPath[MAX_PATH];
    char        * p_debugfs;

    p_debugfs = findDebugfs();
    if (p_debugfs != NULL)
    {
        strcpy(sPath, p_debugfs);
        strcat(sPath,"/tracing/tracing_on");
        if ((iFtraceEnableFd = open(sPath, O_WRONLY)) < 0)
        {
            return -1;
        }
        else
        {
            strcpy(sPath, p_debugfs);
            strcat(sPath,"/tracing/trace_marker");
            if ((iFtraceMarkerFd = open(sPath, O_WRONLY)) < 0)
            {
                close(iFtraceEnableFd);
                return -1;
            }
        }
    }
    else
    {
        return -1;
    }
    return 0;
}

//------------------------------------------------------------------------------
// Function:    FtraceClose
//
// Description: FtraceClose() closes the tracing_on and trace_marker files.
//
// Parameters:  N/A
//
// Return:      0, always
//------------------------------------------------------------------------------
int FtraceClose(void)
{
    close(iFtraceEnableFd);
    close(iFtraceMarkerFd);
    return 0;
}

//------------------------------------------------------------------------------
// Function:    FtraceEnable
//
// Description: FtraceEnable() enables or disables ftrace tracing by writing
//              to the tracing_on file.
//
// Parameters:  fEnable_p       1 = enable, 0 = disable
//
// Return:      N/A
//------------------------------------------------------------------------------
void FtraceEnable(int fEnable_p)
{
    if (fEnable_p)
    {
        write(iFtraceEnableFd, "1", 1);
    }
    else
    {
        write(iFtraceEnableFd, "0", 1);
    }
}

//------------------------------------------------------------------------------
// Function:    FtraceWriteTraceMarker
//
// Description: FtraceWriteTraceMarker() writes an ftrace trace marker.
//
// Parameters:  fmt             format string
//              ...             arguments for the format string
//
// Return:      N/A
//------------------------------------------------------------------------------
void FtraceWriteTraceMarker(char *fmt, ...)
{
    va_list argp;
    char    sMessage[128];
    int len;

    len = sprintf(sMessage, "==== ");
    va_start(argp, fmt);
    len += vsprintf(sMessage + len, fmt, argp);
    va_end(argp);

    sprintf (sMessage + len, " ====\n");
    write (iFtraceMarkerFd, sMessage, strlen(sMessage) + 1);
}
#endif


