#!/bin/bash

if [ -z "$FCMM_NUM_THREADS" ]; then
    echo "Please set the environment variable FCMM_NUM_THREADS"
    exit 1
fi

EXECUTABLE=./benchmark_tbb

# Feel free to change the two variables below as needed
NUM_MILLION_OPERATIONS_LIST="8 16 32 64"        # Million of operations
INSERT_PERCENTAGE_LIST="10 20 30 40 50" # % of insert operations

export FCMM_PRINT_AS_ROW=1

# Print table header
echo "Operations   % of         Time       Time     Time     fcmm speedup   fcmm speedup "
echo "(mln)        insertions   (serial)   (TBB)    (fcmm)   over serial    over TBB     "
echo "-----------------------------------------------------------------------------------"

for NUM_MILLION_OPERATIONS in $NUM_MILLION_OPERATIONS_LIST
do
    for INSERT_PERCENTAGE in $INSERT_PERCENTAGE_LIST
    do
        $EXECUTABLE $FCMM_NUM_THREADS $NUM_MILLION_OPERATIONS $INSERT_PERCENTAGE
    done
done

unset FCMM_PRINT_AS_ROW
