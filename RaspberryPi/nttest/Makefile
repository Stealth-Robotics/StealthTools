
include Makefile.inc
               
MODULE   = nttest
OBJ      = $(OBJ_DIR)/$(MODULE)
MAP      = $(OBJ_DIR)/$(MODULE).map
MKDIR_P  = mkdir -p
              
C_SRC    = $(APP_DIR)client.cpp

CPPTEST_SRC	=	./client.cpp

OBJ_FILES	  = 	$(OBJ_DIR)/*.o

C_OBJ    = $(patsubst $(APP_DIR)%.cpp, $(OBJ_DIR)/%.o, $(C_SRC))
CPPTEST_OBJ    = $(patsubst $(APP_DIR)%.cpp, $(OBJ_DIR)/%.o, $(CPPTEST_SRC))

#############################################################################
# The main Targets
#############################################################################

.PHONY: direct 

all : direct compiles

direct:
	$(MKDIR_P) $(OBJ_DIR)/


cpptest:	$(CPPTEST_OBJ)
	$(CC) -o cpptest $(CPPTEST_OBJ) $(CFLAGS) $(INC) -lpthread -L../ntcore-master/bins/ -lntcore -lwiringPi -lstdc++

compiles:	$(C_OBJ) $(EXECUTABLE)
		ar rcs $(OBJ) $(OBJ_FILES)
clean :
	rm -rf $(OBJ_DIR)/
	$(MKDIR_P) $(OBJ_DIR)/

#############################################################################
# How to build the .elf files from the given .o-files
#############################################################################

$(EXECUTABLE) : $(C_OBJ)
	echo Here

#############################################################################
# This are generic targets how to build .o files out of the given .cpp/.s files
#############################################################################

$(OBJ_DIR)/%.o : $(APP_DIR)%.cpp   
	echo Compiling $< ...
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
