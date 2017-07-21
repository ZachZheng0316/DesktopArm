#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
IsArm			= no

TARGET			=	INDFNOX170605
OBJS			=	INDFNOX170605.o

INCLUDEDIRS		+=	-I ../../include
LIBDIRS			+=	-L ../../build
LIBRARIES		=	-ldxl -lm -lrt -lpthread
CCFLAGS			=	$(INCLUDEDIRS) -W -Wall -O2
CXFLAGS			=	$(INCLUDEDIRS) -w -Wall -O2
LDFLAGS			=	$(LIBDIRS) $(LIBRARIES)

ifeq ($(IsArm), yes)
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
	cd ../../build && make clean
	cd ../../build && make IsArm1=$(IsArm)
	$(CX) -o $@ $^ $(LDFLAGS)
	rm -f *~ *.o

%.o: %.c
	$(CC) $(CCFLAGS) -c $? -o $@

%.o: %.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

clean:
	cd ../../build && make clean
	rm -f $(OBJS) $(TARGET)
	rm -f *~
	@echo "file deleted."
