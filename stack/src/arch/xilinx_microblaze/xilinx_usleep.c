/****************************************************************************
  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  target specific functions
                to Xilinx Microblaze CPU without any OS
                usleep implemenation

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

****************************************************************************/

#include "xilinx_usleep.h"

#include "xparameters.h"

void usleep(DWORD useconds) __attribute__((section(".local_memory")));

#ifdef XPAR_MICROBLAZE_CORE_CLOCK_FREQ_HZ
    #define CPU_SPEED_TICKS XPAR_MICROBLAZE_CORE_CLOCK_FREQ_HZ
#else
    #error "There is not CPU speed available! Please check xparameters.h"
#endif

#define CPU_SPEED_MHZ (CPU_SPEED_TICKS/1000000)

void usleep(DWORD useconds)
{
    /* The small loop always takes 1us -> it is adjusted to need 10 iterations with 50Mhz */
    WORD small_loop = 10 * (CPU_SPEED_MHZ/50);

    asm
    (
      "       addik r20, r0, 1         \n\t"    // fill r20 with decrement value
      "outer_loop: rsub %0, r20, %0    \n\t"
      "inner_loop: rsub %1, r20, %1    \n\t"    //1 cycle
      "       nop                      \n\t"    //1 cycle
      "       bnei %1, inner_loop      \n\t"    //3 cycles
      "       add %1, r0, %2           \n\t"
      "       bnei %0, outer_loop      \n\t"
          : /* no output registers */
          : "r"(useconds), "r"(small_loop), "r"(small_loop)
          : "r20"
    );
}

