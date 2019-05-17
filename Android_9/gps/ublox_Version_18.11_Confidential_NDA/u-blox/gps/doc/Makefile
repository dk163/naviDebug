###############################################################################
#
# Copyright (C) u-blox AG
# u-blox AG, Thalwil, Switzerland
#
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose without fee is hereby granted, provided that this entire notice
# is included in all copies of any software which is or includes a copy
# or modification of this software and in all copies of the supporting
# documentation for such software.
#
# THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
# REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
# OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
#
#******************************************************************************
#
#  Project: Android GNSS driver
#
#*******************************************************************************
# * $Id$
# * $HeadURL$
#*******************************************************************************

# configuration
DEBUG				:= 1
DOXYFILE 			:= Doxyfile
OUTPUT_DIRECTORY 	:= .
DOXYGEN				:= doxygen

# make and tools configuration
GREP 	:= grep
SED 	:= sed
CP		:= cp
MV		:= mv
CAT     := cat
RM      := rm
PERL	:= perl
SVNVERSION := svnversion

# check if verbose is set
ifndef VERBOSE
V		:= @
else
CP		+= -v
RM		+= -v
LN		+= -v
MKDIR	+= -v
endif

# get the project related stuff
PROJECT_NAME   := u-blox Android GNSS Driver
PROJECT_NUMBER := $(shell $(SVNVERSION) ../ 2>/dev/null || echo 0)

all:
	@echo "* Running Doxygen for '$(PROJECT_NAME), $(PROJECT_NUMBER)'"
	$(V)($(CAT) $(DOXYFILE); \
		 echo PROJECT_NAME=\"$(PROJECT_NAME)\"; \
		 echo PROJECT_NUMBER=\"$(PROJECT_NUMBER)\"; \
	) | $(DOXYGEN) - > $(OUTPUT_DIRECTORY)/doxygen_output.txt 2>&1
	
clean:
	@echo "* Remove everything"
	$(V)$(RM) -rf html/
	$(V)$(RM) -rf doxygen_*.txt
	$(V)$(RM) -rf doxygen.warn*.txt

