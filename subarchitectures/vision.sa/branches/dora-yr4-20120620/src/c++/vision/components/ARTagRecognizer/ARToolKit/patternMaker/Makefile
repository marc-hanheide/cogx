# Copyright (c) 2002 The University of Utah
# All Rights Reserved. 
# 
# This is an experimental, research release of a component of the
# Alpha_1 Geometric Modelling System, and as such is subject to a
# license agreement with Engineering Geometry Systems, Inc. and may only
# be used under the terms of that agreement.  Any other use is
# prohibited.
# 
# Having received prior authorization from EGS, the University of Utah grants
# permission to use, copy, or modify this software and its
# documentation for educational, research and non-profit purposes,
# without fee, and without a written agreement.  It is explicitly prohibited to
# redistribute this software or any derivative. 
# 
#       IN NO EVENT SHALL THE UNIVERSITY OF UTAH OR ENGINEERING GEOMETRY
#       SYSTEMS, INC.  BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
#       SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
#       PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
#       DOCUMENTATION, EVEN IF THE UNIVERSITY OF UTAH HAVE BEEN ADVISED
#       OF THE POSSIBILITY OF SUCH DAMAGES.
# 
#       THE UNIVERSITY OF UTAH AND ENGINEERING GEOMETRY SYSTEMS, INC.
#       SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
#       TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#       PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN
#       "AS IS" BASIS, AND THE UNIVERSITY OF UTAH AND ENGINEERING
#       GEOMETRY SYSTEMS, INC. HAVE NO OBLIGATIONS TO PROVIDE
#       MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# 
# The above permission is granted provided that this whole notice
# appears in all copies.
# 
# Contacts are: 
# 
#       EMail:  {dejohnso, cohen} @ cs.utah.edu 


CC = gcc
CFLAGS = -Wall 
LIBS = -lm

OBJS = patterns.o

all: pattern

pattern: $(OBJS) 
	 $(CC) $(CFLAGS) $(OBJS) $(LIBS) -o pattern

clean: 
	-rm -f $(OBJS) pattern core *~ *.o


