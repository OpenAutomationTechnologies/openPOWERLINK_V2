/**
********************************************************************************
\file       Cmp_Lcd.h

\brief      Non generic lcd functions for the TERASIC board

Provides all functions which are platform dependent for the application of the
directIO example.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
*******************************************************************************/

#ifndef _INC_CMP_LCD_H_
#define _INC_CMP_LCD_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "EplNmt.h"
#include "EplErrDef.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

void SysComp_LcdInit(void);
void SysComp_LcdSetText(char* Text);
tEplKernel SysComp_LcdSetLine(int LineNum_p);
void SysComp_LcdClear(void);
void SysComp_LcdTest(void);

void SysComp_LcdPrintState(tEplNmtState NmtState_p);
void SysComp_LcdPrintNodeInfo (WORD wNodeId_p);

#endif /* _INC_CMP_LCD_H_ */
