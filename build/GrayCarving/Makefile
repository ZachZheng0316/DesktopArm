#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
IsArm1			= 	no

TARGET			=	libdxl.a
OBJS			=	Dmath.o PacketHandler.o PortHandler.o DataParser.o DeltaAlgorithm.o XQtorH7.o
SOURCES			=	../../src/Dmath.cpp			 \
					../../src/PacketHandler.cpp	 \
					../../src/PortHandler.cpp	 \
					../../src/DataParser.cpp	 \
					../../src/DeltaAlgorithm.cpp \
					../../src/XQtorH7.cpp		 \

INCLUDEDIRS		+=	-I ../../include
LIBRARIES		+=	-lrt
CCFLAGS			=	$(INCLUDEDIRS) -W -Wall -O2
CXFLAGS			=	$(INCLUDEDIRS) -w -Wall -O2
DIR_SRCS		=	../..
DIR_OBJS		=	.

ifeq ($(IsArm1), yes)
	CC			=	arm-linux-gnueabi-gcc
	CX			=	arm-linux-gnueabi-g++
	AR			=	arm-linux-gnueabi-ar
else
	CC			=	gcc
	CX			=	g++
	AR			=	ar
endif


#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): $(OBJS)
	$(AR) rs $@ $^
	rm -f $(DIR_SRCS)/src/*~
	rm -f $(DIR_OBJS)/*~ $(DIR_OBJS)/*.o

$(DIR_OBJS)/%.o: $(DIR_SRCS)/src/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: $(DIR_SRCS)/src/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

clean:
	rm -f $(OBJS) $(TARGET)
	rm -f *~
	@echo "file deleted."
