;--------------------------------
;Include Modern UI

  !include "MUI2.nsh"
  !include "LogicLib.nsh"

;--------------------------------
;Variables

  Var StartMenuFolder

;--------------------------------
;General

  ;Name and file
   Name    "openPOWERLINK PCIe Driver"
   OutFile "openPOWERLINK_setup.exe"
  ;Default installation folder
   InstallDir $PROGRAMFILES\openPOWERLINK_Driver

  ;Get installation folder from registry if available
   InstallDirRegKey HKCU "Software\openPOWERLINK_Driver" ""

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
  !define MUI_STARTMENUPAGE_REGISTRY_KEY "Software\openPOWERLINK_Driver"
  !define MUI_STARTMENUPAGE_REGISTRY_VALUENAME "Start Menu Folder"

;--------------------------------
;Languages

  !insertmacro MUI_LANGUAGE "English"

;Installation Section for Demo_applications
Section "Demo" section1

;Create directories in program files.
    WriteRegStr HKCU "Software\openPOWERLINK_Driver" "" "$INSTDIR"
    SetOutPath "$INSTDIR\Driver"
    File  /r "..\..\..\bin\windows\amd64\drv_ndis_pcie Package\*.*"
    File  /r "..\..\..\bin\windows\amd64\installer-pcie\*.*"
    File  /r "..\..\..\bin\windows\amd64\uninstaller-pcie\*.*"
    SetOutPath "$INSTDIR\Temp"
    File /r "vcredist_x64.exe"

;Create shortcuts
ExecWait "$INSTDIR\Temp\vcredist_x64.exe"
SetOutPath "$INSTDIR\Driver"
ExecWait "$INSTDIR\Driver\installer-pcie.exe"
WriteUninstaller $INSTDIR\uninstaller.exe
!insertmacro MUI_STARTMENU_WRITE_BEGIN Application
CreateDirectory "$SMPROGRAMS\$StartMenuFolder"
CreateShortCut "$SMPROGRAMS\$StartMenuFolder\uninstaller.lnk" "$INSTDIR\uninstaller.exe"
!insertmacro MUI_STARTMENU_WRITE_END
SectionEnd

Section "Uninstall"
SetOutPath "$INSTDIR\Driver"
ExecWait "$INSTDIR\Driver\uninstaller-pcie.exe"
  ; Remove registry keys
   DeleteRegKey HKLM SOFTWARE\openPOWERLINK_Driver
  ; Remove files and uninstaller
   Delete $INSTDIR\uninstall.exe
   Delete "$INSTDIR\*.*"
   RMDir /r "$INSTDIR\*.*"
   RMDir /r "$PROGRAMFILES\openPOWERLINK_Driver\*.*"
  ; Remove shortcuts, if any
   Delete "$SMPROGRAMS\$StartMenuFolder\uninstaller.lnk"
   RMDir /r "$SMPROGRAMS\$StartMenuFolder\"
SectionEnd