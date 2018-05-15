;------------------------------------------------------------------------------
; \file oplk-ndisim-x64.nsi
;
; \brief NSIS Installer script for NDIS intermediate driver
;
; This file contains the NSIS script to create a Windows installer for the NDIS
; intermediate driver. The installer will allow users to install the driver
; using single click.
;------------------------------------------------------------------------------

;------------------------------------------------------------------------------
;Copyright(c) 2018, Kalycito Infotech Private Limited
;All rights reserved.

;Redistribution and use in source and binary forms, with or without
;modification, are permitted provided that the following conditions are met :
;    * Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;    * Redistributions in binary form must reproduce the above copyright
;      notice, this list of conditions and the following disclaimer in the
;      documentation and / or other materials provided with the distribution.
;    * Neither the name of the copyright holders nor the
;      names of its contributors may be used to endorse or promote products
;      derived from this software without specific prior written permission.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
;ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
;WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;DISCLAIMED.IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
;DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
;(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
;ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
;SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


;Include Modern UI

  !include "MUI2.nsh"
  !include "LogicLib.nsh"
  !include "x64.nsh"

;--------------------------------
;Variables

  Var StartMenuFolder

  ;--------------------------------
;General

  ;Name and file
   Name    "openPOWERLINK NDIS IM driver"
   OutFile "..\..\..\bin\windows\amd64\oplk_ndisim.exe"
   VIProductVersion 1.0.0.0
   VIAddVersionKey ProductName "openPOWERLINK NDIS IM driver v2.3.0.0"
   VIAddVersionKey CompanyName "Ethernet POWERLINK Standardization Group (EPSG)"
   VIAddVersionKey ProductVersion 2.3.0.0
   VIAddVersionKey FileDescription "openPOWERLINK NDIS IM driver installer"

  ;Default installation folder
   InstallDir "$PROGRAMFILES\openPOWERLINK NDIS IM Driver"

  ;Get installation folder from registry if available
   InstallDirRegKey HKCU "Software\oplk_ndisim" ""

  ;Request application privileges for Windows 7 & WinXP
   RequestExecutionLevel admin

   ;Interface Settings

  !define MUI_ABORTWARNING

  ;--------------------------------
;Pages
;page Components InitComponentsPage


  !insertmacro MUI_PAGE_COMPONENTS
  !insertmacro MUI_PAGE_DIRECTORY

  !insertmacro MUI_PAGE_STARTMENU Application $StartMenuFolder

  !insertmacro MUI_PAGE_INSTFILES


  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_COMPONENTS
  !insertmacro MUI_UNPAGE_INSTFILES

;Start Menu Folder Page Configuration
  !define MUI_STARTMENUPAGE_REGISTRY_ROOT "HKCU"
  !define MUI_STARTMENUPAGE_REGISTRY_KEY "Software\openPOWERLINK NDIS IM Driver"
  !define MUI_STARTMENUPAGE_REGISTRY_VALUENAME "Start Menu Folder"

 !define DIR
;Languages

  !insertmacro MUI_LANGUAGE "English"


;Installer Sections

Section "NDIS IM Driver" section1
;Create directories in program files.
    WriteRegStr HKCU "Software\openPOWERLINK NDIS IM Driver" "" "$INSTDIR"
    SetOutPath "$INSTDIR\Driver"
    File  /r "..\..\..\bin\windows\amd64\drv_ndis_intermediate_package\*.*"
    File  /r "..\..\..\bin\windows\amd64\installer-ndisim\*.*"
    SetOutPath "$INSTDIR\Temp"
    File /r "vcredist_x64.exe"

    ExecWait "$INSTDIR\Temp\vcredist_x64.exe /passive /norestart"
    SetOutPath "$INSTDIR\Driver"
    ExecWait "$INSTDIR\Driver\installer-ndisim.exe /Install"
    WriteUninstaller $INSTDIR\uninstaller.exe
    !insertmacro MUI_STARTMENU_WRITE_BEGIN Application
    CreateDirectory "$SMPROGRAMS\$StartMenuFolder"
    CreateShortCut "$SMPROGRAMS\$StartMenuFolder\uninstaller.lnk" "$INSTDIR\uninstaller.exe"
    !insertmacro MUI_STARTMENU_WRITE_END

SectionEnd

Section "Uninstall"
SetOutPath "$INSTDIR\Driver"
ExecWait "$INSTDIR\Driver\installer-ndisim.exe /Uninstall"
  ; Remove registry keys
   DeleteRegKey HKLM "SOFTWARE\openPOWERLINK NDIS IM Driver"
  ; Remove files and uninstaller
   Delete $INSTDIR\uninstall.exe
   Delete "$INSTDIR\*.*"
   RMDir /r "$INSTDIR\*.*"
   RMDir /r "$PROGRAMFILES\openPOWERLINK NDIS IM Driver\*.*"
  ; Remove shortcuts, if any
   Delete "$SMPROGRAMS\$StartMenuFolder\uninstaller.lnk"
   RMDir /r "$SMPROGRAMS\$StartMenuFolder\"
SectionEnd
