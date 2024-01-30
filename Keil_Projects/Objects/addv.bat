@echo off
set /p vn= "Enter Version Number :"

del *.lnp
del *.axf
del *.o
del *.d
del *.htm
del *.dep
del *.map

rename *F421.hex *F421_%vn%.hex
rename *F415.hex *F415_%vn%.hex
rename *F051.hex *F051_%vn%.hex
rename *F031.hex *F031_%vn%.hex
rename *G071.hex *G071_%vn%.hex
rename *E230.hex *E230_%vn%.hex