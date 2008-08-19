/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.de

  Project:      EPL Messe Demo

  Description:  demoapplication for EPL

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2008/04/11 m.u.:   start of implementation

****************************************************************************/

#include <QWidget>
#include <QThread>
#include <QString>

#include <EplApi.h>


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define DEFAULT_MAX_CYCLE_COUNT 20  // 6 is very fast
#define DEFAULT_MODE            0x01
#define LED_COUNT               5       // number of LEDs in one row
#define LED_MASK                ((1 << LED_COUNT) - 1)
#define DOUBLE_LED_MASK         ((1 << (LED_COUNT * 2)) - 1)
#define MODE_COUNT              4
#define MODE_MASK               ((1 << MODE_COUNT) - 1)

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------

EplDataInOutThread  *pEplDataInOutThread_g;

BYTE    abPiIn[4];
BYTE    abPiOut[4];

DWORD   dwMode_l;           // current mode
int     iCurCycleCount_l;   // current cycle count
int     iMaxCycleCount_l;   // maximum cycle count (i.e. number of cycles until next light movement step)
int     iToggle;            // indicates the light movement direction
DWORD   dwLeds_l;           // current state of all LEDs

BYTE    bVarOut1_l;
BYTE    bLedsRow1_l;        // current state of the LEDs in row 1
BYTE    bLedsRow2_l;        // current state of the LEDs in row 2

//=========================================================================//
//                                                                         //
//          E p l D a t a I n O u t T h r e a d                            //
//                                                                         //
//=========================================================================//

EplDataInOutThread::EplDataInOutThread()
{
    pEplDataInOutThread_g = this;

    EplPiIn.m_pImage  = abPiIn;
    EplPiIn.m_uiSize  = sizeof (abPiIn);
    EplPiOut.m_pImage = abPiOut;
    EplPiOut.m_uiSize  = sizeof (abPiIn);

    dwMode_l = DEFAULT_MODE;
    iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;

    bVarOut1_l = 0;
    bLedsRow1_l = 0;
    bLedsRow2_l = 0;
}

void EplDataInOutThread::run()
{
tEplKernel          EplRet;
BYTE    bVarIn1_l;
BYTE    abSelect_l[3];      // pushbuttons from CNs
BYTE    bModeSelect_l;      // state of the pushbuttons to select the mode
BYTE    bSpeedSelect_l;     // state of the pushbuttons to increase/decrease the speed
unsigned int uiLeds;

    do
    {

        EplRet = EplApiProcessImageExchangeIn(&EplPiIn);
        if (EplRet != kEplSuccessful)
        {
            break;
        }

        bVarIn1_l = abPiIn[0];
        abSelect_l[0] = abPiIn[1];
        abSelect_l[1] = abPiIn[2];
        abSelect_l[2] = abPiIn[3];

        // collect inputs from CNs and own input
        bSpeedSelect_l = bVarIn1_l | abSelect_l[0];

        bModeSelect_l = abSelect_l[1] | abSelect_l[2];

        if ((bModeSelect_l & MODE_MASK) != 0)
        {
            dwMode_l = bModeSelect_l & MODE_MASK;
        }

        iCurCycleCount_l--;

        if (iCurCycleCount_l <= 0)
        {
            if ((dwMode_l & 0x01) != 0)
            {   // fill-up
                if (iToggle)
                {
                    if ((dwLeds_l & DOUBLE_LED_MASK) == 0x00)
                    {
                        dwLeds_l = 0x01;
                    }
                    else
                    {
                        dwLeds_l <<= 1;
                        dwLeds_l++;
                        if (dwLeds_l >= DOUBLE_LED_MASK)
                        {
                            iToggle = 0;
                        }
                    }
                }
                else
                {
                    dwLeds_l <<= 1;
                    if ((dwLeds_l & DOUBLE_LED_MASK) == 0x00)
                    {
                        iToggle = 1;
                    }
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x02) != 0)
            {   // running light forward
                dwLeds_l <<= 1;
                if ((dwLeds_l > DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 0x01;
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x04) != 0)
            {   // running light backward
                dwLeds_l >>= 1;
                if ((dwLeds_l > DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 1 << (LED_COUNT * 2);
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x08) != 0)
            {   // Knightrider
                if (bLedsRow1_l == 0x00)
                {
                    bLedsRow1_l = 0x01;
                }
                else if (iToggle)
                {
                    bLedsRow1_l <<= 1;
                    if( bLedsRow1_l >= 0x10 )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    bLedsRow1_l >>= 1;
                    if( bLedsRow1_l <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
                bLedsRow2_l = bLedsRow1_l;
            }

            // set own output
            bVarOut1_l = bLedsRow1_l;
//            bVarOut1_l = (bLedsRow1_l & 0x03) | (bLedsRow2_l << 2);

            // restart cycle counter
            iCurCycleCount_l = iMaxCycleCount_l;
        }

        abPiOut[0] = bVarOut1_l;
        abPiOut[1] = bLedsRow1_l;
        abPiOut[2] = bLedsRow2_l;

        EplRet = EplApiProcessImageExchangeOut(&EplPiOut);

        uiLeds = bLedsRow1_l | (bLedsRow2_l << LED_COUNT);

        // display of MN shows both CN rows
        if (uiLeds != uiLedsOld)
        {
            emit processImageChanged(uiLeds);
            uiLedsOld = uiLeds;
        }


    }
    while (EplRet == kEplSuccessful);
}



