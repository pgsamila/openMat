!define OPENMAT_VERSION_NAME "OpenMAT-1.3.4-dev-20150111"
# !define QT_BASE_DIRECTORY "C:\Qt\Qt5.2.1-vs2010\5.2.1\msvc2010_opengl"
!define QT_BASE_DIRECTORY "C:\Qt\Qt5.2.1\5.2.1\msvc2010_opengl"

Name ${OPENMAT_VERSION_NAME}
OutFile "${OPENMAT_VERSION_NAME}-Setup.exe"
InstallDir "C:\OpenMAT\${OPENMAT_VERSION_NAME}"
InstallDirRegKey HKLM "Software\OpenMAT\${OPENMAT_VERSION_NAME}" "Install_Dir"
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
	File "..\LpSensor\ImuData.h"
	File "..\LpSensor\LpmsDefinitions.h"
	File "..\LpSensor\DeviceListItem.h"
	
	SetOutPath $INSTDIR\bin
	File "..\LpmsControl\build\Release\LpmsControl.exe"
	File "..\LpMocap\build\Release\LpMocap.exe"
	File "..\LpmsControl\LpSplash.png"
	File "..\LpmsControl\README"
	File "..\LpSensor\build\LpSensor.dll"	
	File "..\LpmsControl\Icon.ico"
	File "..\LpmsControl\LpmsStyles.qss"
	File "${QT_BASE_DIRECTORY}\bin\Qt5Core.dll"
	File "${QT_BASE_DIRECTORY}\bin\Qt5Widgets.dll"	
	File "${QT_BASE_DIRECTORY}\bin\Qt5Gui.dll"	
	File "${QT_BASE_DIRECTORY}\bin\Qt5OpenGL.dll"	
	File "${QT_BASE_DIRECTORY}\bin\Qt5Svg.dll"	
	File "${QT_BASE_DIRECTORY}\bin\Qt5Network.dll"
	File "${QT_BASE_DIRECTORY}\bin\Qt5PrintSupport.dll"
	File "${QT_BASE_DIRECTORY}\bin\icudt51.dll"
	File "${QT_BASE_DIRECTORY}\bin\icuin51.dll"
	File "${QT_BASE_DIRECTORY}\bin\icuuc51.dll"
	File "C:\ftdi\i386\ftd2xx.dll"
	File "C:\qwt-6.1.0\lib\qwt.dll"
	File "C:\pcan-basic\Win32\PCANBasic.dll"
	File "C:\opencv\build\x86\vc10\bin\opencv_core248.dll"
	File "C:\opencv\build\x86\vc10\bin\opencv_ffmpeg248.dll"	
	File "C:\opencv\build\x86\vc10\bin\opencv_highgui248.dll"	
	File "C:\opencv\build\x86\vc10\bin\opencv_imgproc248.dll"
	
	SetOutPath $INSTDIR\bin\platforms	
	File "${QT_BASE_DIRECTORY}\plugins\platforms\qwindows.dll"
	
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
	File "..\LpmsControl\icons\loop_alt2_32x28.png"
	File "..\LpmsControl\icons\pause_24x32.png"
	File "..\LpmsControl\icons\compass_32x32.png"
	File "..\LpmsControl\icons\denied_32x32.png"
	
	WriteRegStr HKLM SOFTWARE\OpenMAT\${OPENMAT_VERSION_NAME} "Install_Dir" "$INSTDIR"
	
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT\${OPENMAT_VERSION_NAME}" "DisplayName" "${OPENMAT_VERSION_NAME}"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT\${OPENMAT_VERSION_NAME}" "UninstallString" '"$INSTDIR\uninstall.exe"'
	WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT\${OPENMAT_VERSION_NAME}" "NoModify" 1
	WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT\${OPENMAT_VERSION_NAME}" "NoRepair" 1
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
	CreateDirectory "$SMPROGRAMS\OpenMAT\${OPENMAT_VERSION_NAME}"
	CreateShortCut "$SMPROGRAMS\OpenMAT\${OPENMAT_VERSION_NAME}\Uninstall.lnk" "$INSTDIR\uninstall.exe" "" "$INSTDIR\uninstall.exe" 0
	CreateShortCut "$SMPROGRAMS\OpenMAT\${OPENMAT_VERSION_NAME}\LpmsControl.lnk" "$INSTDIR\bin\LpmsControl.exe" "" "$INSTDIR\bin\Icon.ico" 0
	CreateShortCut "$SMPROGRAMS\OpenMAT\${OPENMAT_VERSION_NAME}\LpMocap.lnk" "$INSTDIR\bin\LpMocap.exe" "" "$INSTDIR\bin\Icon.ico" 0
SectionEnd

Section "Uninstall"
	Delete $DESKTOP\LpmsControl.lnk
	
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenMAT\${OPENMAT_VERSION_NAME}"
	DeleteRegKey HKLM SOFTWARE\OpenMAT\${OPENMAT_VERSION_NAME}

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

	RMDir "$SMPROGRAMS\OpenMAT\${OPENMAT_VERSION_NAME}"
	RMDir "$INSTDIR"
	
	Delete $SMPROGRAMS\OpenMAT\*.*
	RMDir $SMPROGRAMS\OpenMAT
SectionEnd
