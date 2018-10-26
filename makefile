#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#*                                                                           *
#*                  This file is part of the program
#*                                                                           *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

#@file    Makefile
#@brief   Makefile for C++ ALNS
#@author  Florian Grenouilleau
#-----------------------------------------------------------------------------
# Main Program
#-----------------------------------------------------------------------------
SRCDIR = src
INCDIR = include
BINDIR = bin
OBJDIR = obj
LIBDIR = lib
EXT	   = cpp
EXEC = $(BINDIR)/MAPD
LIB	 = $(LIBDIR)/libALNS.a
ifeq ($(DEBUG), TRUE)
   LIB 			+= $(LIBDIR)/libALNS.dbg.a
endif
SRC = $(wildcard $(SRCDIR)/*.$(EXT))
OBJFILES0 = $(patsubst $(SRCDIR)%,$(OBJDIR)%,$(SRC))
MAINOBJFILES = $(OBJFILES0:.$(EXT)=.o)

#-----------------------------------------------------------------------------
# OPTIONS
#-----------------------------------------------------------------------------
DEBUG  = FALSE
OS = $(shell uname -s)

#-----------------------------------------------------------------------------
# default flags
#-----------------------------------------------------------------------------
INCLUDESFLAGS 	= -I$(INCDIR) -I/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/cplex/include -I/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/concert/include -I/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/cpoptimizer/include -I./include/boost/include
FLAGS 			=
CXXFLAGS		=
LIBS  			=
LIBOPTIONS		=
#-----------------------------------------------------------------------------
# add user flags
#-----------------------------------------------------------------------------
CXXFLAGS    	+= -Wall -fPIC -fexceptions -std=c++11  -DNDEBUG -DIL_STD  $(INCLUDESFLAGS)
ifeq ($(DEBUG), TRUE)
   CXXFLAGS 	+= -g -O0
else
   CXXFLAGS 	+= -O3
endif
LIBOPTIONS		+= rvs # -shared -o #rvs
#-----------------------------------------------------------------------------
# Default compiler parameters
#-----------------------------------------------------------------------------
CXX      =  g++
LINKCXX     =  g++
#create a library
AR = ar # g++ # ar

#for linux
ifeq ($(OS),Linux)
   CXX	= g++
   LINKCXX     =  g++
endif

#-----------------------------------------------------------------------------
# Rules
#-----------------------------------------------------------------------------
.PHONY: all
all: $(BINDIR) $(OBJDIR) $(EXEC) 


.PHONY: library
library: $(LIBDIR) $(OBJDIR) $(LIB)

.PHONY: clean
clean:
	@rm -rf $(OBJDIR)/*.o
	@rm -rf $(SRCDIR)/*.o
	@rm -f $(EXEC)
	@rm -rf $(OBJDIR)
	@rm -rf $(BINDIR)

$(EXEC): $(MAINOBJFILES) 
	$(LINKCXX) $(CXXFLAGS) -o $@ $^ -L/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/cplex/lib/x86-64_osx/static_pic -lilocplex -lcplex -L/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/cpoptimizer/lib/x86-64_osx/static_pic -lcp -L/Users/floriangrenouilleau/Applications/IBM/ILOG/CPLEX_Studio1271/concert/lib/x86-64_osx/static_pic -lconcert -lm -lpthread

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@echo "-> compiling $(OBJDIR) $@"
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(LIB): $(MAINOBJFILES)
	$(AR) $(LIBOPTIONS) $(LIB) $(MAINOBJFILES)


$(OBJDIR):
		@mkdir -p $(OBJDIR)

$(BINDIR):
		@mkdir -p $(BINDIR)

$(LIBDIR):
		@mkdir -p $(LIBDIR)
