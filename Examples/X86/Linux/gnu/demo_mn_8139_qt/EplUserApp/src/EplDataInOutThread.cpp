/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for process data thread (input and output).
                This thread implements the control loop.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2008/04/11 m.u.:   start of the implementation

****************************************************************************/

#include <QWidget>
#include <QThread>
#include <QString>

#include <EplApi.h>


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define DEFAULT_MAX_CYCLE_COUNT 20  // 6 is very fast
#define APP_DEFAULT_MODE        0x01
#define APP_LED_COUNT           5       // number of LEDs in one row
#define APP_LED_MASK            ((1 << APP_LED_COUNT) - 1)
#define APP_DOUBLE_LED_MASK     ((1 << (APP_LED_COUNT * 2)) - 1)
#define APP_MODE_COUNT          5
#define APP_MODE_MASK           ((1 << APP_MODE_COUNT) - 1)

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------


// process images
static tAppProcessImageIn   PiIn_g;
static tAppProcessImageOut  PiOut_g;

static DWORD   dwMode_l;           // current mode
static int     iCurCycleCount_l;   // current cycle count
static int     iMaxCycleCount_l;   // maximum cycle count (i.e. number of cycles until next light movement step)
static int     iToggle;            // indicates the light movement direction
static DWORD   dwLeds_l;           // current state of all LEDs

/*
BYTE    bVarOut1_l;
BYTE    bLedsRow1_l;        // current state of the LEDs in row 1
BYTE    bLedsRow2_l;        // current state of the LEDs in row 2
*/

//=========================================================================//
//                                                                         //
//          E p l D a t a I n O u t T h r e a d                            //
//                                                                         //
//=========================================================================//

EplDataInOutThread::EplDataInOutThread()
{
    m_EplPiIn.m_pImage  = &PiIn_g;
    m_EplPiIn.m_uiSize  = sizeof (PiIn_g);
    m_EplPiOut.m_pImage = &PiOut_g;
    m_EplPiOut.m_uiSize  = sizeof (PiOut_g);

    dwMode_l = APP_DEFAULT_MODE;
    iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;

    memset(&PiIn_g, 0, sizeof (PiIn_g));
    memset(&PiOut_g, 0,  sizeof (PiOut_g));
/*
    bVarOut1_l = 0;
    bLedsRow1_l = 0;
    bLedsRow2_l = 0;
*/
}

void EplDataInOutThread::run()
{
tEplKernel          EplRet;
/*
BYTE    bVarIn1_l;
BYTE    abSelect_l[3];      // pushbuttons from CNs
*/
BYTE    bModeSelect_l;      // state of the pushbuttons to select the mode
BYTE    bSpeedSelect_l;     // state of the pushbuttons to increase/decrease the speed
unsigned int uiLeds;
unsigned int uiLedsOld = 0;
unsigned int uiInput;
unsigned int uiInputOld = 0;

    do
    {

        EplRet = EplApiProcessImageExchangeIn(&m_EplPiIn);
        if (EplRet != kEplSuccessful)
        {
            break;
        }
/*
        bVarIn1_l = abPiIn[0];
        abSelect_l[0] = abPiIn[1];
        abSelect_l[1] = abPiIn[2];
        abSelect_l[2] = abPiIn[3];
*/
        // collect inputs from CNs and own input
        bSpeedSelect_l = (PiIn_g.m_bVarIn1 | PiIn_g.m_abSelect[0]) & 0x07;

        bModeSelect_l = PiIn_g.m_abSelect[1] | PiIn_g.m_abSelect[2];

        if ((bModeSelect_l & APP_MODE_MASK) != 0)
        {
            dwMode_l = bModeSelect_l & APP_MODE_MASK;
        }

        iCurCycleCount_l--;

        if (iCurCycleCount_l <= 0)
        {
            if ((dwMode_l & 0x01) != 0)
            {   // fill-up
                if (iToggle)
                {
                    if ((dwLeds_l & APP_DOUBLE_LED_MASK) == 0x00)
                    {
                        dwLeds_l = 0x01;
                    }
                    else
                    {
                        dwLeds_l <<= 1;
                        dwLeds_l++;
                        if (dwLeds_l >= APP_DOUBLE_LED_MASK)
                        {
                            iToggle = 0;
                        }
                    }
                }
                else
                {
                    dwLeds_l <<= 1;
                    if ((dwLeds_l & APP_DOUBLE_LED_MASK) == 0x00)
                    {
                        iToggle = 1;
                    }
                }
                PiOut_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiOut_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x02) != 0)
            {   // running light forward
                dwLeds_l <<= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 0x01;
                }
                PiOut_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiOut_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x04) != 0)
            {   // running light backward
                dwLeds_l >>= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 1 << (APP_LED_COUNT * 2);
                }
                PiOut_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiOut_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x08) != 0)
            {   // Knightrider
                if (PiOut_g.m_bLedsRow1 == 0x00)
                {
                    PiOut_g.m_bLedsRow1 = 0x01;
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    PiOut_g.m_bLedsRow1 <<= 1;
                    if ( PiOut_g.m_bLedsRow1 >= (1 << (APP_LED_COUNT - 1)) )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    PiOut_g.m_bLedsRow1 >>= 1;
                    if( PiOut_g.m_bLedsRow1 <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
                PiOut_g.m_bLedsRow2 = PiOut_g.m_bLedsRow1;
            }

            else if ((dwMode_l & 0x10) != 0)
            {   // Knightrider
                if ((PiOut_g.m_bLedsRow1 == 0x00)
                    || (PiOut_g.m_bLedsRow2 == 0x00)
                    || ((PiOut_g.m_bLedsRow2 & ~APP_LED_MASK) != 0))
                {
                    PiOut_g.m_bLedsRow1 = 0x01;
                    PiOut_g.m_bLedsRow2 = (1 << (APP_LED_COUNT - 1));
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    PiOut_g.m_bLedsRow1 <<= 1;
                    PiOut_g.m_bLedsRow2 >>= 1;
                    if ( PiOut_g.m_bLedsRow1 >= (1 << (APP_LED_COUNT - 1)) )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    PiOut_g.m_bLedsRow1 >>= 1;
                    PiOut_g.m_bLedsRow2 <<= 1;
                    if ( PiOut_g.m_bLedsRow1 <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
            }
            // set own output
            PiOut_g.m_bVarOut1 = PiOut_g.m_bLedsRow1;
//            bVarOut1_l = (bLedsRow1_l & 0x03) | (bLedsRow2_l << 2);

            // restart cycle counter
            iCurCycleCount_l = iMaxCycleCount_l;
        }
/*
        abPiOut[0] = bVarOut1_l;
        abPiOut[1] = bLedsRow1_l;
        abPiOut[2] = bLedsRow2_l;
*/
        uiInput = ((PiIn_g.m_abSelect[1] | PiIn_g.m_abSelect[2]) & APP_LED_MASK)
                  | ((PiIn_g.m_abSelect[0] & APP_LED_MASK) << APP_LED_COUNT);

        // leave sync callback function by passing the output process image to the stack
        EplRet = EplApiProcessImageExchangeOut(&m_EplPiOut);

        uiLeds = PiOut_g.m_bLedsRow1 | (PiOut_g.m_bLedsRow2 << APP_LED_COUNT);

        // display of MN shows both CN rows
        if (uiInput != uiInputOld)
        {
            emit processImageInChanged(uiInput);
            uiInputOld = uiInput;
        }

        if (uiLeds != uiLedsOld)
        {
            emit processImageOutChanged(uiLeds);
            uiLedsOld = uiLeds;
        }


    }
    while (EplRet == kEplSuccessful);
}



