IDIR := ../onboard/include
SDIR := ../onboard/src
ODIR := ./build
BDIR := .
EXE := $(BDIR)/convertfpbin
SRC := $(SDIR)/shared.cpp $(SDIR)/matrix.cpp convertfpbin.cpp
OBJ := $(SRC:$(SDIR)/%.cpp=$(ODIR)/%.o)

CC :=g++
CPPFLAGS := -I$(IDIR) -MMD -MP
CFLAGS := -Wall -std=c++11
LDFLAGS := -Llib

.PHONY: all clean

all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $(CPPFLAGS) $(LDFLAGS) $^ -o $@

$(ODIR)/%.o: $(SDIR)/%.cpp | $(ODIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BDIR) $(ODIR):
	mkdir -p $@

clean:
	@$(RM) -rv $(ODIR)
