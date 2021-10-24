# invoke SourceDir generated makefile for main.pem3
main.pem3: .libraries,main.pem3
.libraries,main.pem3: package/cfg/main_pem3.xdl
	$(MAKE) -f C:\Users\LOOIKA~1\ONEDRI~1\NUS\Year3Sem1\CS3237\cs3237project\src/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\LOOIKA~1\ONEDRI~1\NUS\Year3Sem1\CS3237\cs3237project\src/src/makefile.libs clean

