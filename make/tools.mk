###############################################################
#
# Installers for tools
#
###############################################################


ifeq ($(OS),Windows_NT)
ARM_SDK_PREFIX:=tools/windows/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
SHELL:=cmd.exe
CP:=tools\\windows\\make\\bin\\cp
DSEP:=\\
NUL:=NUL
MKDIR:=tools\\windows\\make\\bin\\mkdir
RM:=tools\\windows\\make\\bin\\rm
CUT:=tools\\windows\\make\\bin\\cut
FGREP:=tools\\windows\\make\\bin\\fgrep
OSDIR:=windows

else
# MacOS and Linux
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
ARM_SDK_PREFIX:=tools/macos/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
CP:=cp
DSEP:=/
NUL:=/dev/null
MKDIR:=mkdir
RM:=rm
CUT:=cut
FGREP:=fgrep
OSDIR:=macos
else
# assume Linux
ARM_SDK_PREFIX:=tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
CP:=cp
DSEP:=/
NUL:=/dev/null
MKDIR:=mkdir
RM:=rm
CUT:=cut
FGREP:=fgrep
OSDIR:=linux
endif
endif

# workaround for lack of a lowercase function in GNU make
# look away before this sends you blind ....
lc = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))

define get_targets
$(shell $(FGREP) "define FILE_NAME" Inc/targets.h | $(FGREP) -v DISABLE_BUILD | $(FGREP) -v "//#" | $(FGREP) _$(1) | $(CUT) -d\" -f2)
endef
