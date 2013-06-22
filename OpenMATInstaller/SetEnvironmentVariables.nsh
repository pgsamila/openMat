!define BINARY_DIR "$INSTDIR\bin"
 
!include WriteEnvStr.nsh
 
Section "Add Env Var"
	!ifdef ALL_USERS
		!define ReadEnvStr_RegKey \
		   'HKLM "SYSTEM\CurrentControlSet\Control\Session Manager\Environment"'
		!else
		!define ReadEnvStr_RegKey 'HKCU "Environment"'
	!endif
 
	Push JAVA_HOME
	Push '${JAVA_HOME}'
	Call WriteEnvStr
	Push APP_HOME
	Push '${APP_HOME}'
	Call WriteEnvStr
	 
	ReadEnvStr $R0 "PATH"
	messagebox mb_ok '$R0'
	
	;ensure that is written valid for NT only
	ReadRegStr $0 ${ReadEnvStr_RegKey} 'JAVA_HOME'
	ReadRegStr $1 ${ReadEnvStr_RegKey} 'APP_HOME'
	StrCpy $R0 "$R0;$0;$1"
	
	;or just this
	;StrCpy $R0 "$R0;${JAVA_HOME};${APP_HOME}"
	
	System::Call 'Kernel32::SetEnvironmentVariableA(t, t) i("PATH", R0).r2'
	ReadEnvStr $R0 "PATH"
	messagebox mb_ok '$R0'
	writeuninstaller '$EXEDIR\uninst.exe'
SectionEnd

Section uninstall
	# remove the variable
	Push JAVA_HOME
	Call un.DeleteEnvStr
	Push APP_HOME
	Call un.DeleteEnvStr
SectionEnd