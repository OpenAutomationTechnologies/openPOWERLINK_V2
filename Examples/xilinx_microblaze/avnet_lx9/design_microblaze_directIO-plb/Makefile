#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : Makefile generation
# Autor         : mairt
# Date          : 13.02.2012
# File          : Makefile
# contents      : Creates the Makefile in the design folder of the example.
################################################################################

XPS= xps
TCL_SCRIPT= build.tcl
XMP_FILE= system.xmp


.PHONY: all
all: bitstream

.PHONY: bitstream
bitstream:
	@echo "Invoking: XPS batch mode"
	$(XPS) -nw -scr $(TCL_SCRIPT) $(XMP_FILE)
	@echo Finished building: 'bitstream'
	@echo ' '
