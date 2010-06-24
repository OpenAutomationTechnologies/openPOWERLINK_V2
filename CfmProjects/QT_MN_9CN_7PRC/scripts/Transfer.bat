:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: (c) Kalycito Infotech Private Limited
::
::  Project:		openCONFIGURATOR - written by Kalycito 
::
::	File name:		TransferCDC.bat
::
::  Description:  
::	Copies the file set to "Src' variable to the folder set to "Dest" variable
::  For help on editing this file for the purpose of openCONFIGURATOR, please refer
::  to the User Manual document of openCONFIGURATOR
::
::  License:
::
::   Redistribution and use in source and binary forms, with or without
::   modification, are permitted provided that the following conditions
::   are met:
::
::   1. Redistributions of source code must retain the above copyright
::      notice, this list of conditions and the following disclaimer.
::
::   2. Redistributions in binary form must reproduce the above copyright
::      notice, this list of conditions and the following disclaimer in the
::      documentation and/or other materials provided with the distribution.
::
::   3. Neither the name of Kalycito Infotech Private Limited nor the names of 
::      its contributors may be used to endorse or promote products derived
::      from this software without prior written permission. For written
::      permission, please contact info@kalycito.com.
::
::   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
::   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
::   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
::   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
::   COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
::   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
::   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
::   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
::   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
::   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
::   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
::   POSSIBILITY OF SUCH DAMAGE.
::
::   Severability Clause:
::
::       If a provision of this License is or becomes illegal, invalid or
::       unenforceable in any jurisdiction, that shall not affect:
::       1. the validity or enforceability in that jurisdiction of any other
::          provision of this License; or
::       2. the validity or enforceability in other jurisdictions of that or
::          any other provision of this License.
::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::Syntax:
:: xcopy "Source" "Destination" <Switch(s)>
::
::Example:
:: set CdcSrc="C:\Documents and Settings\UserName\My Documents\openCONFIGURATOR_Projects\Project1\cdc_xap\mnobd.cdc"
:: set CdcDest="C:\Documents and Settings\UserName\Desktop\openPOWERLINK\"
:: 
:: set XapSrc="C:\Documents and Settings\UserName\My Documents\openCONFIGURATOR_Projects\Project1\cdc_xap\xap.h"
:: set XapDest="C:\Documents and Settings\UserName\Desktop\openPOWERLINK\"
::
:: Consult the user manual for Trouble shooting and for more explanation
::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


@echo off

:: Set the Source file
set CdcSrc=""
set XapSrc=""
::echo The cdc Source is "%CdcSrcSrc%"
::echo The xap.h Source is "%XapSrcSrc%"

:: Set the Destination folder file
set CdcDest=""
set XapDest=""
::echo The cdc Destination is "%CdcDest%"
::echo The xap.h Destination is "%XapDest%"

xcopy %CdcSrc% %CdcDest% /Y
xcopy %XapSrc% %XapDest% /Y