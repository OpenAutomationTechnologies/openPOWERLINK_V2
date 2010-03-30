/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Wrapper for POSIX file API for Linux kernel
                (open, read, write, close)

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
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2010/01/12 d.k.:   start of implementation

****************************************************************************/

#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include "PosixFileLinuxKernel.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

int errno;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//  Kernel Module specific Data Structures
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------

FD_TYPE open(const char *filename, int oflags, int pmode)
{
FD_TYPE         hFile;
mm_segment_t    old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    hFile = filp_open(filename, oflags, pmode);

    set_fs(old_fs);

    if (IS_ERR(hFile))
    {
        errno = (int) -PTR_ERR(hFile);
        hFile = NULL;
    }

    return hFile;
}


off_t lseek(FD_TYPE fd, off_t offset, int origin)
{
mm_segment_t    old_fs;
loff_t          pos;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    pos = vfs_llseek(fd, (loff_t) offset, origin);

    set_fs(old_fs);

    return (off_t) pos;
}


ssize_t read(FD_TYPE fd, void *buffer, size_t count)
{
mm_segment_t    old_fs;
loff_t          pos;
ssize_t         iRet;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    pos = fd->f_pos;
    iRet = vfs_read(fd, buffer, count, &pos);
    fd->f_pos = pos;

    set_fs(old_fs);

    if (iRet < 0)
    {
        errno = (int) iRet;
        iRet = -1;
    }

    return iRet;
}


int close(FD_TYPE fd)
{
mm_segment_t    old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    filp_close(fd, current->files);

    set_fs(old_fs);

    return 0;
}


// EOF

