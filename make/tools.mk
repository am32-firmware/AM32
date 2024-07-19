###############################################################
#
# Installers for tools
#
###############################################################


# download location for tools
WINDOWS_TOOLS=http://uav.tridgell.net/AM32/tools/windows-tools.zip
LINUX_TOOLS=http://uav.tridgell.net/AM32/tools/linux-tools.tar.gz

ifeq ($(OS),Windows_NT)
ARM_SDK_PREFIX:=tools/windows/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
SHELL:=cmd.exe
COPY:=copy /Y
DSEP:=\\
NUL:=NUL
MKDIR:=mkdir
RMDIR:=del /s /q

arm_sdk_install:
	@echo "Installing windows tools"
	@$(RMDIR) tools\\windows
	@powershell -Command "& { (New-Object System.Net.WebClient).DownloadFile('$(WINDOWS_TOOLS)', 'windows-tools.zip') }"
	@powershell -Command "Expand-Archive -Path windows-tools.zip -Force -DestinationPath ."
	@echo "windows tools install done"

else
ARM_SDK_PREFIX:=tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
COPY:=cp
DSEP:=/
NUL:=/dev/null
MKDIR:=mkdir -p
RMDIR:=rm -rf

arm_sdk_install:
	@echo "Installing linux tools"
	@wget $(LINUX_TOOLS)
	@tar xzf linux-tools.tar.gz
	@echo "linux tools install done"

endif
