/**
 * Copyright (c) 2013, Giacomo Drago <giacomo@giacomodrago.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes fcmm, a software developed by Giacomo Drago.
 *      Website: http://projects.giacomodrago.com/fcmm
 * 4. Neither the name of Giacomo Drago nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GIACOMO DRAGO "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL GIACOMO DRAGO BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fcmm.hpp"

#include "common.hpp"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <thread>
#include <iomanip>
#include <random>

#include <tbb/concurrent_hash_map.h>

static const std::uint16_t KEY_FIELD_MAX_VALUE = 511;

class KeyHashCompareTbb {
private:
    KeyHash1 keyHash;
public:
    std::size_t hash(const Key& k) const {
        return keyHash(k);
    }
    bool equal(const Key& k1, const Key& k2) const {
        return k1 == k2;
    }
};

typedef std::unordered_map<Key, Value, KeyHash1> StdMapType;
typedef tbb::concurrent_hash_map<Key, Value, KeyHashCompareTbb> TbbConcurrentMapType;
typedef fcmm::Fcmm<Key, Value, KeyHash1, KeyHash2> FcmmType;

void threadFunction(int threadNo, int numOperationsPerThread, float insertOperationsPercent,
                    std::function<void(const Key& key)> find, std::function<void(const Key& key)> insert) {

    std::default_random_engine generator(threadNo);
    std::uniform_int_distribution<std::uint16_t> randNum(0, KEY_FIELD_MAX_VALUE);
    std::uniform_real_distribution<float> randFloat(0.0f, 100.0f);

    for (int i = 0; i < numOperationsPerThread; i++) {
        std::uint16_t a = randNum(generator);
        std::uint16_t b = randNum(generator);
        std::uint16_t c = randNum(generator);
        Key key(a, b, c);
        if (randFloat(generator) < insertOperationsPercent) {
            insert(key);
        } else {
            find(key);
        }
    }

}

void runSerial(int expectedNumEntries, int numOperations, float insertOperationsPercent) {

    StdMapType map(expectedNumEntries);

    auto find = [&map](const Key& key) {
        map.find(key);
    };

    auto insert = [&map](const Key& key) {
        if (map.find(key) == map.end()) {
            Value value = calculate(key);
            map.insert(std::make_pair(key, value));
        }
    };

    runThreads(1, threadFunction, numOperations, insertOperationsPercent, find, insert);

}

void runTbb(int expectedNumEntries, int numThreads, int numOperationsPerThread, float insertOperationsPercent) {

    TbbConcurrentMapType map(expectedNumEntries);

    auto find = [&map](const Key& key) {
        TbbConcurrentMapType::const_accessor accessor;
        map.find(accessor, key);
    };

    auto insert = [&map](const Key& key) {
        TbbConcurrentMapType::accessor accessor;
        if (map.insert(accessor, key)) {
            accessor->second = calculate(key);
        }
    };

    runThreads(numThreads, threadFunction, numOperationsPerThread, insertOperationsPercent, find, insert);

}

void runFcmm(int expectedNumEntries, int numThreads, int numOperationsPerThread, float insertOperationsPercent, bool verbose = false) {

    FcmmType map(expectedNumEntries);

    auto find = [&map](const Key& key) {
        map.find(key);
    };

    auto insert = [&map](const Key& key) {
        map.insert(key, calculate);
    };

    runThreads(numThreads, threadFunction, numOperationsPerThread, insertOperationsPercent, find, insert);

    if (verbose) {
        printStats(map);
    }

}

int main(int argc, char** argv) {

    if (argc != 4) {
        std::cerr << "usage: benchmark_tbb NUM_THREADS NUM_MILLION_OPERATIONS INSERT_OPERATIONS_PERCENT" << std::endl;
        return EXIT_FAILURE;
    }

    int numThreads = atoi(argv[1]);
    float numMillionOperations = atof(argv[2]);
    float insertOperationsPercent = atof(argv[3]);

    int numOperations = numMillionOperations * 1.0e6f;
    int numOperationsPerThread = numOperations / numThreads;
    int expectedNumEntries = numOperations * (insertOperationsPercent / 100.0f);

    char* printAsRowEnv = getenv("FCMM_PRINT_AS_ROW");
    char* verboseEnv = getenv("FCMM_VERBOSE");

    bool printAsRow = printAsRowEnv != NULL && atoi(printAsRowEnv) == 1;
    bool verbose = !printAsRow && verboseEnv != NULL && atoi(verboseEnv) == 1;

    if (!printAsRow) {

        std::cout << std::endl;

        std::cout << "Comparative Benchmark of fcmm and tbb::concurrent_hash_map" << std::endl << std::endl;

        std::cout << "Number of threads: " << numThreads << std::endl
                  << "Number of operations: " << numMillionOperations << " million" << std::endl
                  << "Percentage of insert operations: " << insertOperationsPercent << "%" << std::endl;

        std::cout << std::endl;

    }

    if (!printAsRow) std::cout << "Running benchmark with std::unordered_map (serial execution)..." << std::endl;
    int elapsedSerial = benchmark(runSerial, expectedNumEntries, numOperations, insertOperationsPercent);
    if (!printAsRow) std::cout << "Time elapsed: " << elapsedSerial << " ms." << std::endl << std::endl;

    if (!printAsRow) std::cout << "Running benchmark with tbb::concurrent_hash_map..." << std::endl;
    int elapsedTbb = benchmark(runTbb, expectedNumEntries, numThreads, numOperationsPerThread, insertOperationsPercent);
    if (!printAsRow) std::cout << "Time elapsed: " << elapsedTbb << "ms." << std::endl << std::endl;

    if (!printAsRow) std::cout << "Running benchmark with fcmm..." << std::endl;
    int elapsedFcmm = benchmark(runFcmm, expectedNumEntries, numThreads, numOperationsPerThread, insertOperationsPercent, verbose);
    if (!printAsRow) std::cout << "Time elapsed: " << elapsedFcmm << " ms." << std::endl << std::endl;

    float speedupOverSerial = (float) elapsedSerial / elapsedFcmm;
    float speedupOverTbb = (float) elapsedTbb / elapsedFcmm;

    if (!printAsRow) {
        std::cout << "fcmm speedup over serial execution (std::unordered_map): " << speedupOverSerial << std::endl;
        std::cout << "fcmm speedup over TBB: " << speedupOverTbb << std::endl;
    } else {
        std::cout << std::left << std::fixed << std::setprecision(2)
                  << std::setw(13) << (int) numMillionOperations
                  << std::setw(13) << (int) insertOperationsPercent
                  << std::setw(11) << elapsedSerial
                  << std::setw(9) << elapsedTbb
                  << std::setw(9) << elapsedFcmm
                  << std::setw(15) << speedupOverSerial
                  << std::setw(0) << speedupOverTbb
                  << std::endl;
    }

    return EXIT_SUCCESS;

}
