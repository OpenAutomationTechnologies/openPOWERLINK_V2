/**
********************************************************************************

  \file           EplDataInOutThread.cpp

  \brief          Implementation of the EplDataInOutThread class

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

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

/******************************************************************************/
/* includes */
#include <QWidget>
#include <QThread>
#include <QString>

#include <EplApi.h>

/******************************************************************************/
/* definitions */
#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))


/******************************************************************************/
/* global variables */
EplDataInOutThread    *pEplDataInOutThread_g;

static int iUsedNodeIds_g[] = {1, 32, 110, 0};

/* process images, structures defined in xap.h from openCONFIGURATOR */
static PI_IN                    PiIn_g;
static PI_OUT                   PiOut_g;
tEplApiProcessImageCopyJob      PICopyJob_g;

/******************************************************************************/
/* global functions */

/**
********************************************************************************
\brief  synchronous data callback

AppCbSync() is the openPOWERLINK callback function used to transfer synchronous
data.

\retval         kEplSuccessfull         if function was successfull
\retval         misc. error codes       if an error occured
*******************************************************************************/
tEplKernel PUBLIC AppCbSync(void)
{
    tEplKernel          EplRet;
    int                 i;

    EplRet = EplApiProcessImageExchange(&PICopyJob_g);
    if (EplRet != kEplSuccessful)
    {
        return EplRet;
    }

    pEplDataInOutThread_g->m_uiCnt++;

    pEplDataInOutThread_g->m_uiInput[0] = PiOut_g.CN1_M00_Digital_Input_8_Bit_Byte_1;
    pEplDataInOutThread_g->m_uiInput[1] = PiOut_g.CN32_M00_Digital_Input_8_Bit_Byte_1;
    pEplDataInOutThread_g->m_uiInput[2] = PiOut_g.CN110_M00_Digital_Input_8_Bit_Byte_1;

    for (i = 0; (i < MAX_NODES) && (iUsedNodeIds_g[i] != 0); i++)
    {
        /* Running Leds */
        /* period for LED flashing determined by inputs */
        pEplDataInOutThread_g->m_uiPeriod[i] = (pEplDataInOutThread_g->m_uiInput[i] == 0) ? 1 : (pEplDataInOutThread_g->m_uiInput[i] * 20);
        if (pEplDataInOutThread_g->m_uiCnt % pEplDataInOutThread_g->m_uiPeriod[i] == 0)
        {
            if (pEplDataInOutThread_g->m_uiLeds[i] == 0x00)
            {
                pEplDataInOutThread_g->m_uiLeds[i] = 0x1;
                pEplDataInOutThread_g->m_iToggle[i] = 1;
            }
            else
            {
                if (pEplDataInOutThread_g->m_iToggle[i])
                {
                    pEplDataInOutThread_g->m_uiLeds[i] <<= 1;
                    if (pEplDataInOutThread_g->m_uiLeds[i] == APP_LED_MASK_1)
                    {
                        pEplDataInOutThread_g->m_iToggle[i] = 0;
                    }
                }
                else
                {
                    pEplDataInOutThread_g->m_uiLeds[i] >>= 1;
                    if (pEplDataInOutThread_g->m_uiLeds[i] == 0x01)
                    {
                        pEplDataInOutThread_g->m_iToggle[i] = 1;
                    }
                }
            }
        }

        if (pEplDataInOutThread_g->m_uiInput[i] != pEplDataInOutThread_g->m_uiInputOld[i])
        {
            pEplDataInOutThread_g->inChanged(pEplDataInOutThread_g->m_uiInput[i], iUsedNodeIds_g[i]);
            pEplDataInOutThread_g->m_uiInputOld[i] = pEplDataInOutThread_g->m_uiInput[i];
        }

        if (pEplDataInOutThread_g->m_uiLeds[i] != pEplDataInOutThread_g->m_uiLedsOld[i])
        {
            pEplDataInOutThread_g->outChanged(pEplDataInOutThread_g->m_uiLeds[i], iUsedNodeIds_g[i]);
            pEplDataInOutThread_g->m_uiLedsOld[i] = pEplDataInOutThread_g->m_uiLeds[i];
        }
    }

    PiIn_g.CN1_M00_Digital_Ouput_8_Bit_Byte_1 = pEplDataInOutThread_g->m_uiLeds[0];
    PiIn_g.CN32_M00_Digital_Ouput_8_Bit_Byte_1 = pEplDataInOutThread_g->m_uiLeds[1];
    PiIn_g.CN110_M00_Digital_Ouput_8_Bit_Byte_1 = pEplDataInOutThread_g->m_uiLeds[2];

    return EplRet;
}

/******************************************************************************/
/* member functions */

/**
********************************************************************************
\brief  constructor

Constructs an EplDataInOutThread object.
*******************************************************************************/
EplDataInOutThread::EplDataInOutThread()
{
    int         i;

    memset(&PiIn_g, 0, sizeof (PiIn_g));
    memset(&PiOut_g, 0,  sizeof (PiOut_g));

    /* setup process image copy job */
    PICopyJob_g.m_In.m_pPart      = &PiIn_g;
    PICopyJob_g.m_In.m_uiOffset   = 0;
    PICopyJob_g.m_In.m_uiSize     = sizeof (PiIn_g);
    PICopyJob_g.m_Out.m_pPart     = &PiOut_g;
    PICopyJob_g.m_Out.m_uiOffset  = 0;
    PICopyJob_g.m_Out.m_uiSize    = sizeof (PiOut_g);
    PICopyJob_g.m_uiPriority      = 0;
    PICopyJob_g.m_fNonBlocking    = FALSE;

    /* initialize all application variables */
    for (i = 0; (i < MAX_NODES) && (iUsedNodeIds_g[i] != 0); i++)
    {
        m_uiLeds[i] = 0;
        m_uiLedsOld[i] = 0;
        m_uiInput[i] = 0;
        m_uiInputOld[i] = 0;
        m_iToggle[i] = 0;
        m_uiPeriod[i] = 0;
    }

    pEplDataInOutThread_g = this;
}

/**
********************************************************************************
\brief  setup the process image

SetupProcessImage() sets up the process image used by the application.

\retval         kEplSuccessfull         if process image was successfully setup
\retval         misc. error codes       if process image setup failed
*******************************************************************************/
tEplKernel EplDataInOutThread::SetupProcessImage()
{
tEplKernel          EplRet;

    EplRet = EplApiProcessImageAlloc(sizeof (PiIn_g), sizeof (PiOut_g), 2 , 2);
    if (EplRet != kEplSuccessful)
    {
        printf("EplApiProcessImageAlloc():  0x%X\n", EplRet);
        goto Exit;
    }

    EplRet = EplApiProcessImageSetup();
    if (EplRet != kEplSuccessful)
    {
        printf("EplApiProcessImageSetup():  0x%X\n", EplRet);
        goto Exit;
    }

Exit:
    return EplRet;
}

/**
********************************************************************************
\brief  signal change of input process image

inChanged() signals that there are changes in the input process image.

\param  uiInput_p       input data
\param  iUsedNodeId_p   node ID the data belongs to
*******************************************************************************/
void EplDataInOutThread::inChanged(unsigned int uiInput_p, int iUsedNodeId_p)
{
    emit processImageInChanged(uiInput_p, iUsedNodeId_p);
}

/**
********************************************************************************
\brief  signal change of output process image

outChanged() signals that there are changes in the output process image.

\param  uiLed_p         output data
\param  iUsedNodeId_p   node ID the data belongs to
*******************************************************************************/
void EplDataInOutThread::outChanged(unsigned int uiLed_p, int iUsedNodeId_p)
{
    emit processImageOutChanged(uiLed_p, iUsedNodeId_p);
}

/**
********************************************************************************
\brief  thread starting point

run() implements the starting point for the data input/output thread.
*******************************************************************************/
void EplDataInOutThread::run()
{
tEplKernel  EplRet;

    printf("EplDataInOutThread started\n");
    for (;;)
    {
        EplRet = AppCbSync();
        if (EplRet != kEplSuccessful)
        {
            printf("EplDataInOutThread stopped with 0x%X\n", EplRet);
            return;
        }
        QThread::msleep(8);
    }
}

/**
********************************************************************************
\brief  returns pointer to callback

getSyncCbFunc() returns the address of the synchronous data callback function.

\return address of synchronous data callback function
*******************************************************************************/
tEplSyncCb EplDataInOutThread::getSyncCbFunc()
{
    return AppCbSync;
}
