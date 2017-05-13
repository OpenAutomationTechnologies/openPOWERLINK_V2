/**
********************************************************************************
\file   sim-target.h

\brief  Include file for simulation interface providing target functions

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
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
#ifndef _INC_sim_target_H_
#define _INC_sim_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <sim.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

OPLKDLLEXPORT BOOL  sim_setTargetFunctions(tSimulationInstanceHdl simHdl_p,
                                           tTargetFunctions targetFunctions_p);
OPLKDLLEXPORT void  sim_unsetTargetFunctions(void);

tOplkError          sim_initTarget(void);
tOplkError          sim_exitTarget(void);
void                sim_msleep(UINT32 milliSeconds_p);
tOplkError          sim_setIpAdrs(const char* ifName_p,
                                  UINT32 ipAddress_p,
                                  UINT32 subnetMask_p,
                                  UINT16 mtu_p);
tOplkError          sim_setDefaultGateway(UINT32 defaultGateway_p);
UINT32              sim_getTickCount(void);
tOplkError          sim_setLed(tLedType ledType_p, BOOL fLedOn_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_sim_target_H_ */
