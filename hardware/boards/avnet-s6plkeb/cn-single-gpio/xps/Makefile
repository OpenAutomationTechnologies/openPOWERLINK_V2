#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : Build system initialization
# File          : Makefile
# contents      : Generates the Makefile in the design folder of the example by
#                 opening xps in batch mode.
################################################################################

XPS= xps
TCL_SCRIPT= build.tcl
XMP_FILE= system.xmp
SYSTEM_MAKE= system.make


.PHONY: all
all: $(SYSTEM_MAKE)

$(SYSTEM_MAKE):
	@echo "Invoking: XPS batch mode"
	$(XPS) -nw -scr $(TCL_SCRIPT) $(XMP_FILE)
	@echo Finished generating: '$(SYSTEM_MAKE)'
	@echo ' '
