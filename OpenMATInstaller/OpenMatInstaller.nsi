Name "OpenMAT 1.1.2"
OutFile "OpenMAT-1.1.2-Setup.exe"
InstallDir "C:\OpenMAT"
InstallDirRegKey HKLM "Software\OpenMAT" "Install_Dir"
RequestExecutionLevel admin

!define MUI_ICON "${NSISDIR}\Contrib\Graphics\Icons\orange-install.ico"
!define MUI_UNICON "${NSISDIR}\Contrib\Graphics\Icons\orange-uninstall.ico"

!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_RIGHT
!define MUI_HEADERIMAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Header\orange-r.bmp"
!define MUI_HEADERIMAGE_UNBITMAP "${NSISDIR}\Contrib\Graphics\Header\orange-uninstall-r.bmp"
 
!define MUI_WELCOMEFINISHPAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Wizard\orange.bmp"
!define MUI_UNWELCOMEFINISHPAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Wizard\orange-uninstall.bmp"

!include "MUI2.nsh"

!define MUI_ABORTWARNING

!insertmacro MUI_PAGE_LICENSE "COPYING"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
  
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
 
!insertmacro MUI_LANGUAGE "English"

/* !include "drvinstall.nsh" */

/* UninstPage uninstConfirm
UninstPage instfiles */

BrandingText "LP-Research Installer"

Section "OpenMAT (required)"
	SectionIn RO

	SetOutPath $INSTDIR\lib
	SetOutPath $INSTDIR\lib\x86
	
	File "..\LpSensor\build\LpSensorD.lib"
	File "..\LpSensor\build\LpSensorD.dll"
	File "..\LpSensor\build\LpSensor.lib"
	File "..\LpSensor\build\LpSensor.dll"
	
	File "..\Libraries\Ftdi\i386\ftd2xx.dll"
	/* File "..\Libraries\PcanApi\Win32\PCANBasic.dll" */	
	
	/* SetOutPath $INSTDIR\lib\x64	
	File "..\LpSensor\build\x64\LpSensorD64.lib"
	File "..\LpSensor\build\x64\LpSensorD64.dll"
	File "..\LpSensor\build\x64\LpSensor64.lib"
	File "..\LpSensor\build\x64\LpSensor64.dll"
	
	File "..\Libraries\Ftdi\amd64\ftd2xx64.dll"	*/
	
	SetOutPath $INSTDIR\examples\simple
	File "..\LpmsSimpleExample\main.cpp"
	File "..\LpmsSimpleExample\CMakeLists.txt"
	
	SetOutPath $INSTDIR\include	
	
	File "..\LpSensor\LpmsSensorI.h"
	File "..\LpSensor\LpmsSensorManagerI.h"
	File "..\OpenMATCommon\ImuData.h"
	File "..\LpSensor\LpmsDefinitions.h"
	File "..\LpSensor\DeviceListItem.h"
	
	SetOutPath $INSTDIR\bin

	File "..\LpmsControl\build\Release\LpmsControl.exe"
	# File "..\LpmsControl\*.xml"
	File "..\LpmsControl\LpSplash.png"
	
	File "..\LpmsControl\README"
	
	File "..\LpSensor\build\LpSensor.dll"	
	
	/* File "C:\Users\klaus\lp-research\ProductDevelopment\branches\SW\LpmsControl\LpmsCase.obj" */
	
	File "..\LpmsControl\Icon.ico"
	File "..\LpmsControl\LpmsStyles.qss"

	File "..\OpenMATCommon\StartOpenMATServer.bat"
	File "..\OpenMATCommon\config.icebox"
	File "..\OpenMATCommon\config.service"
		
	/* File "C:\Qt\4.7.4\bin\phonon4.dll" */
	File "C:\Qt\4.8.2\bin\QtCore4.dll"
	File "C:\Qt\4.8.2\bin\QtGui4.dll"
	File "C:\Qt\4.8.2\bin\QtOpenGL4.dll"
	/* File "C:\Qt\4.7.4\bin\QtXml4.dll" */
	File "C:\Qt\4.8.2\bin\QtSvg4.dll"
	
	/* File "C:\Program Files (x86)\ZeroC\Ice-3.4.2\bin\vc100\ice34.dll"
	File "C:\Program Files (x86)\ZeroC\Ice-3.4.2\bin\vc100\icestorm34.dll"
	File "C:\Program Files (x86)\ZeroC\Ice-3.4.2\bin\vc100\bzip2.dll"	
		
	File "C:\Program Files (x86)\ZeroC\Ice-3.4.2\bin\vc100\iceutil34.dll" */
		
	File "..\Libraries\Ftdi\i386\ftd2xx.dll"
		
	File "..\Libraries\Qwt\lib\qwt.dll"
	
	File "..\Libraries\PcanApi\Win32\PCANBasic.dll"	
	
	SetOutPath $INSTDIR\bin\icons
	File "..\LpmsControl\icons\stop_32x32.png"
	File "..\LpmsControl\icons\play_24x32.png"
	File "..\LpmsControl\icons\x_alt_32x32.png"
	File "..\LpmsControl\icons\cd_32x32.png"
	File "..\LpmsControl\icons\bolt_32x32.png"
	File "..\LpmsControl\icons\x_28x28.png"
	File "..\LpmsControl\icons\plus_32x32.png"
	File "..\LpmsControl\icons\cd_32x32.png"
	File "..\LpmsControl\icons\rss_alt_32x32.png"
	File "..\LpmsControl\icons\target_32x32.png"
	File "..\LpmsControl\icons\share_32x32.png"
	File "..\LpmsControl\icons\sun_fill_32x32.png"
	File "..\LpmsControl\icons\layers_32x28.png"
	File "..\LpmsControl\icons\eyedropper_32x32.png"
	File "..\LpmsControl\icons\folder_stroke_32x32.png"
	File "..\LpmsControl\icons\fullscreen_exit_32x32.png"
	File "..\LpmsControl\icons\bars_32x32.png"
	File "..\LpmsControl\icons\user_24x32.png"
	
	WriteRegStr HKLM SOFTWARE\OpenMAT "Install_Dir" "$INSTDIR"
	
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT" "DisplayName" "OpenMAT"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT" "UninstallString" '"$INSTDIR\uninstall.exe"'
	WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT" "NoModify" 1
	WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT" "NoRepair" 1
	WriteUninstaller "uninstall.exe"
	
	SetOutPath $INSTDIR		   
    File "..\Drivers\Ftdi\CDM20824_Setup.exe"
    ExecWait "$INSTDIR\CDM20824_Setup.exe"
    ExecWait "$INSTDIR\CDM20824_Setup.exe"
    Delete "$INSTDIR\CDM20824_Setup.exe"
	
	SetOutPath $INSTDIR		   
    File "..\Drivers\VC2010Redistributable\vcredist_x86.exe"
    ExecWait "$INSTDIR\vcredist_x86.exe /q"         
    Delete $INSTDIR\vcredist_x86.exe 
SectionEnd

Section "Start Menu Shortcuts"
	SetOutPath $INSTDIR\Bin
	CreateDirectory "$SMPROGRAMS\OpenMAT"
	CreateShortCut "$SMPROGRAMS\OpenMAT\Uninstall.lnk" "$INSTDIR\uninstall.exe" "" "$INSTDIR\uninstall.exe" 0
	CreateShortCut "$SMPROGRAMS\OpenMAT\LpmsControl.lnk" "$INSTDIR\bin\LpmsControl.exe" "" "$INSTDIR\bin\Icon.ico" 0
	/* CreateShortCut "$SMPROGRAMS\OpenMAT\HumanBodySimulation.lnk" "$INSTDIR\bin\HumanBodySimulation.exe" "" "$INSTDIR\bin\Icon.ico" 0 */
	/* CreateShortCut "$SMPROGRAMS\OpenMAT\StartOpenMATServer.lnk" "$INSTDIR\bin\StartOpenMATServer.bat" "" "$INSTDIR\bin\Icon.ico" 0 */
	/* CreateShortCut "$SMPROGRAMS\OpenMAT\LpmsSimpleExample.lnk" "$INSTDIR\bin\LpmsSimpleExample.exe" "" "$INSTDIR\bin\Icon.ico" 0
	CreateShortCut "$SMPROGRAMS\OpenMAT\OpenMATTemplateApp.lnk" "$INSTDIR\bin\OpenMATTemplateApp.exe" "" "$INSTDIR\bin\Icon.ico" 0 */ 
SectionEnd

Section "Desktop Icon"
	CreateShortCut "$DESKTOP\LpmsControl.lnk" "$INSTDIR\bin\LpmsControl.exe" "" "$INSTDIR\bin\Icon.ico" 0
SectionEnd

Section "Uninstall"
	Delete $DESKTOP\LpmsControl.lnk
	
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT"
	DeleteRegKey HKLM SOFTWARE\OpenMAT

	Delete $INSTDIR\bin\icons\green\*.*
	RMDir $INSTDIR\bin\icons\green		

	Delete $INSTDIR\bin\icons\orange\*.*
	RMDir $INSTDIR\bin\icons\orange			

	Delete $INSTDIR\bin\icons\*.*
	RMDir $INSTDIR\bin\icons	
	
	Delete $INSTDIR\bin\*.*
	RMDir $INSTDIR\bin
	
	Delete $INSTDIR\include\*.*
	RMDir $INSTDIR\include
	
	Delete $INSTDIR\*.*
	RMDir $INSTDIR\

	Delete $INSTDIR\lib\x86\*.*
	RMDir $INSTDIR\lib\x86
	Delete $INSTDIR\lib\*.*
	RMDir $INSTDIR\lib

	RMDir "$SMPROGRAMS\OpenMAT"
	RMDir "$INSTDIR"
	
	Delete $SMPROGRAMS\OpenMAT\*.*
	RMDir $SMPROGRAMS\OpenMAT
SectionEnd
