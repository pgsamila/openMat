Name "OpenMAT 1.2.6"
OutFile "OpenMAT-1.2.6-Setup.exe"
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

!insertmacro MUI_PAGE_LICENSE "..\LpmsControl\README"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
  
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
 
!insertmacro MUI_LANGUAGE "English"

BrandingText "LP-RESEARCH Installer"

Section "OpenMAT (required)"
	SectionIn RO

	SetOutPath $INSTDIR\lib
	SetOutPath $INSTDIR\lib\x86
	
	File "..\LpSensor\build\LpSensorD.lib"
	File "..\LpSensor\build\LpSensorD.dll"
	File "..\LpSensor\build\LpSensor.lib"
	File "..\LpSensor\build\LpSensor.dll"
	
	File "c:\ftdi\i386\ftd2xx.dll"
	File "c:\qwt-6.1.0\lib\qwt.dll"
	File "c:\pcan-basic\Win32\PCANBasic.dll"
	
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
	File "..\LpmsControl\LpSplash.png"
	File "..\LpmsControl\README"
	File "..\LpSensor\build\LpSensor.dll"	
	File "..\LpmsControl\Icon.ico"
	File "..\LpmsControl\LpmsStyles.qss"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5Core.dll"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5Widgets.dll"	
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5Gui.dll"	
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5OpenGL.dll"	
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5Svg.dll"	
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5Network.dll"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\Qt5PrintSupport.dll"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\icudt51.dll"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\icuin51.dll"
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\bin\icuuc51.dll"
	
	File "c:\ftdi\i386\ftd2xx.dll"
	File "c:\qwt-6.1.0\lib\qwt.dll"
	File "c:\pcan-basic\Win32\PCANBasic.dll"	
	
	SetOutPath $INSTDIR\bin\platforms	
	File "C:\Qt\Qt5.1.1\5.1.1\msvc2010_opengl\plugins\platforms\qwindows.dll"
	
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
