/**
 * Copyright (c) 2014, Giacomo Drago <giacomo@giacomodrago.com>
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

#include <fcmm.hpp>

#include "common.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <thread>

static const std::uint16_t KEY_FIELD_MAX_VALUE = 255;
static const std::size_t EXPECTED_NUM_ENTRIES = (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1); // about 16 million
static const int TEST_RUNS = 10;
static const int NUM_THREADS = 4;

typedef fcmm::Fcmm<Key, Value, KeyHash1, KeyHash2> FcmmType;

void threadFunction(int threadNo, FcmmType& map, std::array<int, NUM_THREADS>& performedInsertions) {
    std::array<std::uint16_t, KEY_FIELD_MAX_VALUE+1> sequence;
    for (std::size_t i = 0; i < sequence.size(); i++) {
        sequence[i] = i;
    }
    std::shuffle(sequence.begin(), sequence.end(), std::minstd_rand(threadNo));
    for (std::uint16_t a : sequence) {
        for (std::uint16_t b : sequence) {
            for (std::uint16_t c : sequence) {
                const Key key(a, b, c);
                const auto insertResult = map.insert(key, calculate);
                assert(insertResult.first->first == key, "Inconsistent keys after insertion [1]");
                const auto findResult = map.find(key);
                assert(findResult != map.end(), "Could not find the inserted entry immediately after insertion");
                assert(findResult->first == key, "Inconsistent keys after insertion [2]");
                const Value& insertedValue = insertResult.first->second;
                const Value& foundValue = findResult->second;
                const Value expectedValue = calculate(key);
                assert(insertedValue == foundValue && insertedValue == expectedValue, "Inconsistent values after insertion");
                if (insertResult.second) {
                    performedInsertions[threadNo]++;
                }
            }
        }
    }
}

void checkConsistency(const FcmmType& map) {
    assert(!map.empty(), "The map should not be empty");
    assert(map.size() == map.getNumEntries(), "Inconsistent map size");
    for (std::uint16_t a = 0; a <= KEY_FIELD_MAX_VALUE; a++) {
        for (std::uint16_t b = 0; b <= KEY_FIELD_MAX_VALUE; b++) {
            for (std::uint16_t c = 0; c <= KEY_FIELD_MAX_VALUE; c++) {
                const Key key(a, b, c);
                const auto findResult = map.find(key);
                assert(findResult != map.end(), "Could not find a key");
                const Value& actualValue = findResult->second;
                const Value expectedValue = calculate(key);
                assert(actualValue == expectedValue, "Inconsistent value found");
            }
        }
    }
}

void checkIterator(const FcmmType& map, std::size_t performedInsertionsTotal) {
    std::size_t counter = 0;
    FcmmType::const_iterator it;
    Key nextKey = Key();
    Value nextValue = Value();
    bool first = true;
    for (it = map.begin(); it != map.end(); ++it) {
        using std::swap;
        const FcmmType::Entry& entry = *it;
        auto findResult = map.find(entry.first);
        assert(findResult != map.end(), "Inconsistent iterator [1]");
        assert(it->second == findResult->second, "Inconsistent iterator [2]");
        if (!first) {
            assert(it->first == nextKey, "Inconsistent iterator postincrement [1]");
            assert(it->second == nextValue, "Inconsistent iterator postincrement [2]");
        } else {
            first = false;
        }
        auto itBackup = it;
        assert(itBackup == it++, "Inconsistent iterator postincrement [3]");
        if (it != map.end()) {
            nextKey = it->first;
            nextValue = it->second;
        }
        swap(it, itBackup);
        counter++;
    }
    assert(counter == performedInsertionsTotal, "Inconsistent iterator [3] (counter: " + std::to_string(counter) + ")");
}

void checkClone(const FcmmType& map, std::size_t performedInsertionsTotal, std::size_t duplicates) {
    std::unique_ptr<FcmmType> mapClone(map.clone());
    assert(mapClone->size() == performedInsertionsTotal - duplicates, "Inconsistent clone() behavior [1]"); // clone() should remove duplicates
    for (FcmmType::const_iterator it = mapClone->begin(); it != mapClone->end(); ++it) {
        const FcmmType::Entry& entry = *it;
        assert(entry.second == map.at(entry.first), "Inconsistent clone() behavior [2]");
    }
    checkConsistency(*mapClone);
}

void checkFilter(const FcmmType& map) {

    auto filterFunction = [](const FcmmType::Entry& entry) {
        return entry.first.a == 0;
    };

    std::unique_ptr<FcmmType> filteredMap(map.filter(filterFunction));

    assert(filteredMap->getNumEntries() == (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1), "Inconsistent filter() behavior [1]");

    for (const auto& entry : *filteredMap) {
        assert(entry.first.a == 0, "Inconsistent filter() behavior [2]");
        assert(entry.second == map[entry.first], "Inconsistent filter() behavior [3]");
    }

    for (std::uint16_t a = 1; a <= KEY_FIELD_MAX_VALUE; a++) {
        for (std::uint16_t b = 0; b <= KEY_FIELD_MAX_VALUE; b++) {
            for (std::uint16_t c = 0; c <= KEY_FIELD_MAX_VALUE; c++) {
                Key key(a, b, c);
                auto findResult = filteredMap->find(key);
                assert(findResult == filteredMap->end(), "Inconsistent filter() behavior [4]");
            }
        }
    }

}

void performTest(bool verbose = false) {


    // Create and populate the map

    FcmmType map;

    assert(map.empty(), "Map should be empty");
    assert(map.size() == 0, "Inconsistent map size [1]");
    assert(map.getNumEntries() == 0, "Inconsistent map size [2]");

    std::array<int, NUM_THREADS> performedInsertions;
    std::fill_n(performedInsertions.begin(), performedInsertions.size(), 0);

    runThreads(NUM_THREADS, threadFunction, std::ref(map), std::ref(performedInsertions));


    // Check number of insertions

    std::size_t performedInsertionsTotal = 0;

    for (int threadNo = 0; threadNo < NUM_THREADS; threadNo++) {
        if (verbose) {
            std::cout << "Thread no: " << threadNo << ", "
                    << "performed insertions: " << performedInsertions[threadNo]
                    << std::endl;
        }
        performedInsertionsTotal += performedInsertions[threadNo];
    }

    const std::size_t duplicates = performedInsertionsTotal - EXPECTED_NUM_ENTRIES;

    if (performedInsertionsTotal >= EXPECTED_NUM_ENTRIES) {
        if (verbose) {
            const double duplicatesPercent = 100.0 * duplicates / performedInsertionsTotal;
            std::cout << "Minimum total insertions: " << EXPECTED_NUM_ENTRIES << ", "
                    << "actual: " << performedInsertionsTotal
                    << std::endl
                    << "The map contains " << duplicates << " duplicates (" << duplicatesPercent << "%)"
                    << std::endl;
        }
    } else {
        fail("Minimum total insertions: " + std::to_string(EXPECTED_NUM_ENTRIES) + ", "
                 + "actual: " + std::to_string(performedInsertionsTotal));
    }

    if (verbose) {
        printStats(map);
    }


    // Check consistency

    std::cout << "Checking consistency: " << std::flush;
    checkConsistency(map);
    std::cout << "OK." << std::endl;


    // Check iterator

    std::cout << "Checking iterator: " << std::flush;
    checkIterator(map, performedInsertionsTotal);
    std::cout << "OK." << std::endl;


    // Check clone()

    std::cout << "Checking clone(): " << std::flush;
    checkClone(map, performedInsertionsTotal, duplicates);
    std::cout << "OK." << std::endl;


    // Check filter()

    std::cout << "Checking filter(): " << std::flush;
    checkFilter(map);
    std::cout << "OK." << std::endl;


}

int main(void) {

    char* verboseEnv = getenv("FCMM_VERBOSE");
    bool verbose = verboseEnv != NULL && atoi(verboseEnv) == 1;

    std::cout
            << "Correctness test for Fast Concurrent Memoization Map (fcmm)" << std::endl << std::endl
            << "The test will be executed " << TEST_RUNS << " times and may require a long time. " << std::endl
            << "Please be patient, this is not a benchmark." << std::endl << std::endl;

    for (int i = 1; i <= TEST_RUNS; i++) {
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Run " << i << " of " << TEST_RUNS << std::endl;
        try {
            performTest(verbose);
        } catch (std::exception& e) {
            fail("Exception: " + std::string(e.what()));
        }
    }

    std::cout << "TEST SUCCEEDED" << std::endl;

    return EXIT_SUCCESS;

}
