NAME = Commander

CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

LD_FLAGS :=
CC_FLAGS := -MMD -Wall  

debug: CC_FLAGS += -g -ggdb
debug: NAME = MAIN_DEBUG
debug: $(NAME)

release: CC_FLAGS += -O3
release: NAME = MAIN_RELEASE
release: $(NAME)

$(NAME): $(OBJ_FILES)
	   g++ $(LD_FLAGS) -o $@ $^

obj/%.o: src/%.cpp
	   g++ $(CC_FLAGS) -c -o $@ $<

clean:
	rm -rf obj/*.o obj/*.d

