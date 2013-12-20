# Use this Makefile to compile tests and examples with g++
# In order to compile benchmark_tbb, the Intel Threading Building Blocks (TBB)
# library is needed.

CC                = g++
CFLAGS_COMMON     = -I".." -std=c++11 -Wall
CFLAGS_RELEASE    = $(CFLAGS_COMMON) -O3 -fexpensive-optimizations
CFLAGS_DEBUG      = $(CFLAGS_COMMON) -O0 -g
CFLAGS            = $(CFLAGS_RELEASE)
LDFLAGS           = -pthread
LDFLAGS_TBB       = -ltbb

all: correctness_test benchmark_tbb move_semantics

correctness_test: correctness_test.o
	$(CC) -o $@ $@.o $(CFLAGS) $(LDFLAGS)

benchmark_tbb: benchmark_tbb.o
	$(CC) -o $@ $@.o $(CFLAGS) $(LDFLAGS) $(LDFLAGS_TBB)

move_semantics: move_semantics.o
	$(CC) -o $@ $@.o $(CFLAGS) $(LDFLAGS) $(LDFLAGS_TBB)

%.o: %.cpp common.hpp ../fcmm.hpp
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	@rm -f correctness_test.o
	@rm -f benchmark_tbb.o
	@rm -f move_semantics.o
	@rm -f correctness_test
	@rm -f benchmark_tbb
	@rm -f move_semantics
