###############################################################
#
# Installers for tools
#
###############################################################


# download location for tools
WINDOWS_TOOLS=https://firmware.ardupilot.org/Tools/AM32-tools/windows-tools.zip
LINUX_TOOLS=https://firmware.ardupilot.org/Tools/AM32-tools/linux-tools.tar.gz

ifeq ($(OS),Windows_NT)
ARM_SDK_PREFIX:=tools/windows/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
SHELL:=cmd.exe
COPY:=tools\\windows\\make\\bin\\cp -f
DSEP:=\\
NUL:=NUL
MKDIR:=tools\\windows\\make\\bin\\mkdir -p
RM:=tools\\windows\\make\\bin\\\rm -rf

arm_sdk_install:
	@echo "Installing windows tools"
	@$(RM) -rf tools\\windows
	@powershell -Command "& { (New-Object System.Net.WebClient).DownloadFile('$(WINDOWS_TOOLS)', 'windows-tools.zip') }"
	@powershell -Command "Expand-Archive -Path windows-tools.zip -Force -DestinationPath ."
	@echo "windows tools install done"

else
ARM_SDK_PREFIX:=tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
COPY:=cp -f
DSEP:=/
NUL:=/dev/null
MKDIR:=mkdir -p
RM:=rm -rf

arm_sdk_install:
	@echo "Installing linux tools"
	@wget $(LINUX_TOOLS)
	@tar xzf linux-tools.tar.gz
	@echo "linux tools install done"

endif

# workaround for lack of a lowercase function in GNU make
# look away before this sends you blind ....
lc = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))
