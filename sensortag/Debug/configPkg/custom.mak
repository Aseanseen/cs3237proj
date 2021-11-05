## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em3 linker.cmd package/cfg/main_pem3.oem3

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/main_pem3.xdl
	$(SED) 's"^\"\(package/cfg/main_pem3cfg.cmd\)\"$""\"C:/Users/Looi Kai Wen/OneDrive - National University of Singapore/NUS/Year3Sem1/CS3237/cs3237project/src/Debug/configPkg/\1\""' package/cfg/main_pem3.xdl > $@
	-$(SETDATE) -r:max package/cfg/main_pem3.h compiler.opt compiler.opt.defs
