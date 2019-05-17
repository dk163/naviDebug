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
# Purpose: Batch file to build example.exe with SSL support without 
#          changing source code files
#
###############################################################################

SET CL=/DUSE_SSL
MSBuild.exe "Example.sln" /p:DefineConstants=USE_SSL /property:Configuration=Release /property:Platform=Win32 /m /t:Rebuild