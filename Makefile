CC   = clang++ #required clang for gnu extensions
INC  = 
LIBS = -fopenmp=libomp
FLG  = -Wall \
	   -Wextra \
	   -pedantic \
	   -Wno-gnu-anonymous-struct \
	   -Wno-gnu-case-range \
	   -Wno-nested-anon-types \
	   -O2 \
	   -std=gnu++20
	   
DEF  = -D_CRT_SECURE_NO_WARNINGS
OUT  = jpg2ppm.exe

all: $(OUT)

OUT_SRCS := $(wildcard *.cpp)
OUT_OBJS := $(patsubst %.cpp, build/%.o, $(OUT_SRCS))
OUT_DEPS := $(patsubst %.cpp, build/%.d, $(OUT_SRCS))

$(OUT): $(OUT_OBJS)
	$(CC) $^ $(LIBS) $(FLG) -o $(OUT)
	
-include $(OUT_DEPS)

build/%.o: ./%.cpp
	$(CC) $(FLG) $(LIBS) $(DEF) $(INC) -MMD -c $< -o $@
	
clean:
	rm -f $(OUT_OBJS) $(OUT_DEPS) $(OUT)
	
run: $(OUT)
	$(OUT)