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
    dwMode_l = APP_DEFAULT_MODE;
    iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;

    memset(&PiIn_g, 0, sizeof (PiIn_g));
    memset(&PiOut_g, 0,  sizeof (PiOut_g));
}


tEplKernel EplDataInOutThread::SetupProcessImage()
{
tEplKernel          EplRet;
unsigned int        uiVarEntries;

    EplRet = EplApiProcessImageAlloc(sizeof (PiIn_g), sizeof (PiOut_g), 2 , 2);
    if (EplRet != kEplSuccessful)
    {
        printf("EplApiProcessImageAlloc():  0x%X\n", EplRet);
        goto Exit;
    }

    // link process variables to OD
    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x6000, 0x01, memberoffs(tAppProcessImageOut, m_bVarIn1), TRUE, sizeof (PiOut_g.m_bVarIn1), &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    uiVarEntries = 3;
    EplRet = EplApiProcessImageLinkObject(0x2200, 0x01, memberoffs(tAppProcessImageOut, m_abSelect), TRUE, sizeof (PiOut_g.m_abSelect[0]), &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x6200, 0x01, memberoffs(tAppProcessImageIn, m_bVarOut1), FALSE, sizeof (PiIn_g.m_bVarOut1), &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x2000, 0x01, memberoffs(tAppProcessImageIn, m_bLedsRow1), FALSE, sizeof (PiIn_g.m_bLedsRow1), &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x2000, 0x02, memberoffs(tAppProcessImageIn, m_bLedsRow2), FALSE, sizeof (PiIn_g.m_bLedsRow2), &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return EplRet;
}


void EplDataInOutThread::run()
{
tEplKernel          EplRet;
BYTE    bModeSelect_l;      // state of the pushbuttons to select the mode
BYTE    bSpeedSelect_l;     // state of the pushbuttons to increase/decrease the speed
BYTE    bSpeedSelectOld_l = 0;
unsigned int uiLeds;
unsigned int uiLedsOld = 0;
unsigned int uiInput;
unsigned int uiInputOld = 0;

tEplApiProcessImageCopyJob  PICopyJob;

    PICopyJob.m_In.m_pPart      = &PiIn_g;
    PICopyJob.m_In.m_uiOffset   = 0;
    PICopyJob.m_In.m_uiSize     = sizeof (PiIn_g);
    PICopyJob.m_Out.m_pPart     = &PiOut_g;
    PICopyJob.m_Out.m_uiOffset  = 0;
    PICopyJob.m_Out.m_uiSize    = sizeof (PiOut_g);
    PICopyJob.m_uiPriority      = 0;
    PICopyJob.m_fNonBlocking    = FALSE;

    for (;;)
    {
        EplRet = EplApiProcessImageExchange(&PICopyJob);
        if (EplRet != kEplSuccessful)
        {
            break;
        }

        // collect inputs from CNs and own input
        bSpeedSelect_l = (PiOut_g.m_bVarIn1 | PiOut_g.m_abSelect[0]) & 0x07;

        bModeSelect_l = PiOut_g.m_abSelect[1] | PiOut_g.m_abSelect[2];

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
                PiIn_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiIn_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x02) != 0)
            {   // running light forward
                dwLeds_l <<= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 0x01;
                }
                PiIn_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiIn_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x04) != 0)
            {   // running light backward
                dwLeds_l >>= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 1 << (APP_LED_COUNT * 2);
                }
                PiIn_g.m_bLedsRow1 = (unsigned char) (dwLeds_l & APP_LED_MASK);
                PiIn_g.m_bLedsRow2 = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x08) != 0)
            {   // Knightrider
                if (PiIn_g.m_bLedsRow1 == 0x00)
                {
                    PiIn_g.m_bLedsRow1 = 0x01;
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    PiIn_g.m_bLedsRow1 <<= 1;
                    if ( PiIn_g.m_bLedsRow1 >= (1 << (APP_LED_COUNT - 1)) )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    PiIn_g.m_bLedsRow1 >>= 1;
                    if( PiIn_g.m_bLedsRow1 <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
                PiIn_g.m_bLedsRow2 = PiIn_g.m_bLedsRow1;
            }

            else if ((dwMode_l & 0x10) != 0)
            {   // Knightrider
                if ((PiIn_g.m_bLedsRow1 == 0x00)
                    || (PiIn_g.m_bLedsRow2 == 0x00)
                    || ((PiIn_g.m_bLedsRow2 & ~APP_LED_MASK) != 0))
                {
                    PiIn_g.m_bLedsRow1 = 0x01;
                    PiIn_g.m_bLedsRow2 = (1 << (APP_LED_COUNT - 1));
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    PiIn_g.m_bLedsRow1 <<= 1;
                    PiIn_g.m_bLedsRow2 >>= 1;
                    if ( PiIn_g.m_bLedsRow1 >= (1 << (APP_LED_COUNT - 1)) )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    PiIn_g.m_bLedsRow1 >>= 1;
                    PiIn_g.m_bLedsRow2 <<= 1;
                    if ( PiIn_g.m_bLedsRow1 <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
            }
            // set own output
            PiIn_g.m_bVarOut1 = PiIn_g.m_bLedsRow1;

            // restart cycle counter
            iCurCycleCount_l = iMaxCycleCount_l;
        }

        if (bSpeedSelectOld_l == 0)
        {
            if ((bSpeedSelect_l & 0x01) != 0)
            {
                if (iMaxCycleCount_l < 200)
                {
                    iMaxCycleCount_l++;
                }
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
            else if ((bSpeedSelect_l & 0x02) != 0)
            {
                if (iMaxCycleCount_l > 1)
                {
                    iMaxCycleCount_l--;
                }
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
            else if ((bSpeedSelect_l & 0x04) != 0)
            {
                iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
        }
        else if (bSpeedSelect_l == 0)
        {
            bSpeedSelectOld_l = 0;
        }

        uiInput = ((PiOut_g.m_abSelect[1] | PiOut_g.m_abSelect[2]) & APP_LED_MASK)
                  | ((PiOut_g.m_abSelect[0] & APP_LED_MASK) << APP_LED_COUNT);

        uiLeds = PiIn_g.m_bLedsRow1 | (PiIn_g.m_bLedsRow2 << APP_LED_COUNT);

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


        QThread::msleep(8);
    }
}



