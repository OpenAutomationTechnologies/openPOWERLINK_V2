/*******************************************************************************

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Description:  Library to provide console IO primitives
                (kbhit, getch, etc.) on different plattforms.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
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

#include "Epl.h"
#include "global.h"
#include <EplTgtConio.h>

// Supported plattform:
// _MSC_VER .... Microsoft Compiler (Visual Studio)
// __linux__ ... Linux

#ifdef _MSC_VER

    #include <conio.h>

#elif __linux__

    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>

#endif

//=========================================================================//
// Type definitions                                                        //
//=========================================================================//

//=========================================================================//
// Module internal variables                                               //
//=========================================================================//

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:        EplTgtGetch
//
// Description:     read current character from STDIN, no echo
//
// Parameters:      void
//
// Returns:         input from STDIN or EOF on end of file of error
//
//---------------------------------------------------------------------------

#ifdef _MSC_VER

    // Use Microsoft's _getch() (ISO C++) to provided in conio.h
    int EplTgtGetch( void )
    {
        return  _getch();
    }

#elif __linux__

    // Use termios library to provide getch()
    int EplTgtGetch( void )
    {
        struct  termios oldt;
        struct  termios newt;
        int             ch;

        // Save old attributes
        tcgetattr( STDIN_FILENO, &oldt );

        // Calculate and set new attributes
        // ECHO ..... echo input key on/off
        // ICANON ... line buffering on/off
        newt            = oldt;
        newt.c_lflag    &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt );

        // Read input key
        ch = getchar();

        // Restore old attributes
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

        return ch;
    }

#endif

//---------------------------------------------------------------------------
//
// Function:        EplTgtKbhit
//
// Description:     Check if an input character is available in STDIN
//
// Parameters:      void
//
// Returns:         0 ... no input available
//                  1 ... input available
//
//---------------------------------------------------------------------------

#ifdef _MSC_VER

    // Use Microsoft's _kbhit() (ISO C++) to provided in conio.h
    int EplTgtKbhit( void )
    {
        return  _kbhit();
    }

#elif __linux__

    // Use termios library to provide getch()
    int EplTgtKbhit( void )
    {
        struct  termios oldt;
        struct  termios newt;
        int             oldf;
        int             newf;
        int             ch;

        // Save current attributes and file status
        tcgetattr(STDIN_FILENO, &oldt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

        // Calculate new attributes and file status
        newt            = oldt;
        newt.c_lflag    &= ~(ICANON | ECHO);
        newf            = oldf | O_NONBLOCK;

        // Set new attributes and file status
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, newf);

        // Read input
        ch = getchar();

        // Restore save attributes and file status
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        // Put read character back where it was taken
        if(ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }

#endif

//---------------------------------------------------------------------------
//
// Function:        MilliSleep
//
// Description:     Sleep for the specified amount of milli seconds
//
// Parameters:      Milli seconds to sleep
//
// Returns:         void
//
//---------------------------------------------------------------------------

#ifdef _MSC_VER

    // Use Microsoft's Sleep implementation
    void    EplTgtMilliSleep( unsigned int MilliSeconds )
    {
        Sleep(MilliSeconds);
    }

#elif __linux__

    // Use Microsoft's Sleep implementation
    void    EplTgtMilliSleep( unsigned int MilliSeconds )
    {
        struct  timeval timeout;
        fd_set          readFds;
        int             maxFd;
        int             iSelectRetVal;
        unsigned int    Seconds;
        unsigned int    MicroSeconds;

        // initialize file descriptor set
        maxFd   = 0 + 1;
        FD_ZERO(&readFds);

        // Calculate timeout values
        Seconds         = MilliSeconds / 1000;
        MicroSeconds    = (MilliSeconds - (Seconds * 1000)) * 1000;

        // initialize timeout value
        timeout.tv_sec  = Seconds;
        timeout.tv_usec = MicroSeconds;

        iSelectRetVal = select(maxFd, &readFds, NULL, NULL, &timeout);
        switch (iSelectRetVal)
        {
            case 0:     // select timeout occurred, no packet received
                        break;

            case -1:    // select error occurred
                        break;

            default:    // packet available for receive
                        break;
        }
    }

#endif
