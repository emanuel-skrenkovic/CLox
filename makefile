cc = gcc
flags=-Wall # -Werror
extra=-Wextra

c_files := $(wildcard ./src/*.c)
h_files := $(wildcard ./src/*.h)

./bin/%.o: %.c $(h_files)
	$(cc) -c -o $@ $<
	
./bin/main: $(c_files)
	$(cc) -g -o $@ $^ $(flags) $(extra)

run: ./bin/main
	@./bin/main $(t)

test: ./bin/main
	find tests/ -maxdepth 1 -type f -exec ./bin/main {} \;
