/**
********************************************************************************
\file   oplk/version.h

\brief  openPOWERLINK version definitions

The file contains definitions describing the openPOWERLINK version.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_oplk_version_H_
#define _INC_oplk_version_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

// NOTE:
// All version macros should contain the same version number. But do not use
// defines instead of the numbers. Because the macro PLK_STRING_VERSION() can not
// convert a define to a string. In case of a release candidate (rc) version
// the PLK_STRING_VERSION() must be replaced with PLK_STRING_VERSION_RC() in order
// to add the "-rc" suffix.
//
// Format: maj.min.build-rc
//         maj                  = major version
//             min              = minor version (will be set to 0 if major version will be incremented)
//                 build        = current build (will be set to 0 if minor version will be incremented)
//                       rc     = release candidate (will be set to 0 if final version created)
//
#define PLK_DEFINED_STACK_VERSION   PLK_STACK_VERSION       (2, 7, 1, 0)
#define PLK_DEFINED_OBJ1018_VERSION PLK_OBJ1018_VERSION     (2, 7, 1, 0)
#define PLK_DEFINED_STRING_VERSION  PLK_STRING_VERSION      (2, 7, 1, 0)

#endif /* _INC_oplk_version_H_ */
