###############################################################################
#
# Copyright (C) u-blox AG
# u-blox AG, Thalwil, Switzerland
#
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose without fee is hereby granted, provided that this entire notice is
# included in all copies of any software which is or includes a copy or
# modification of this software and in all copies of the supporting
# documentation for such software.
#
# THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
# REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
# THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
#
###############################################################################
#
# Project: libMGA 
# Purpose: Makefile to create example executable with or without SSL support
#
###############################################################################

.DEFAULT_GOAL := example


CPPFLAGS = "-I../src" "-I./Parser" -g -Wall -Wextra -Wno-unused-parameter -DUNIX_API -ggdb -fstrict-aliasing


objects = example.o libMga.o parserbuffer.o protocolnmea.o protocolubx.o protocolunknown.o LinuxSupport.o arguments.o

objectsSSL = example.o libMgaSSL.o parserbuffer.o protocolnmea.o protocolubx.o protocolunknown.o LinuxSupport.o arguments.o

SSLFLAGS = -L../src/mbedtls/bin -lmbedtls -lmbedx509 -lmbedcrypto -lpthread

# listing of all available targets
help:
	@echo "Available targets:"
	@echo ""
	@echo "example   -- Customer 32-bit version of example executable"
	@echo "exampleSSL -- Customer 32-bit version of example executable with SSL support"
	@echo ""


example: $(objects)
	g++ -o example $(objects) -lpthread
	
exampleSSL: $(objectsSSL)
	g++ -o example $(objectsSSL) $(SSLFLAGS)	

example.o: example.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

libMga.o: ../src/libMga.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

libMgaSSL.o: ../src/libMga.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) -DUSE_SSL $< -o $@

parserbuffer.o : ./Parser/parserbuffer.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

protocolnmea.o: ./Parser/protocolnmea.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

protocolubx.o: ./Parser/protocolubx.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

protocolunknown.o: ./Parser/protocolunknown.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

LinuxSupport.o: ./linux/LinuxSupport.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@

arguments.o: arguments.cpp
	g++ -c $(CFLAGS) $(CPPFLAGS) $< -o $@


.PHONY: clean
clean:
	$(RM) -f *.o
	$(RM) -f *.gcno
	$(RM) -f example
	$(RM) -r $(ODIR)/


