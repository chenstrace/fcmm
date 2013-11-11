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
#include <array>
#include <algorithm>
#include <random>
#include <thread>

static const std::string REPORT_RECIPIENT = "Giacomo Drago <giacomo@giacomodrago.com>";

static const std::uint16_t KEY_FIELD_MAX_VALUE = 255;
static const std::size_t EXPECTED_NUM_ENTRIES = (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1); // about 16 million
static const int TEST_RUNS = 10;
static const int NUM_THREADS = 4;

void failTest(const std::string& message) {
    std::cerr << std::endl << message << std::endl
              << "TEST FAILED" << std::endl
              << "Please report full test output along with your test environment" << std::endl
              << "to " << REPORT_RECIPIENT << std::endl;
    exit(EXIT_FAILURE);
}

typedef fcmm::Fcmm<Key, Value, KeyHash1, KeyHash2> FcmmType;

void threadFunction(int threadNo, FcmmType& map, std::array<int, NUM_THREADS>& performedInsertions) {
    std::array<std::uint16_t, KEY_FIELD_MAX_VALUE+1> sequence;
    for (std::size_t i = 0; i < sequence.size(); i++) {
        sequence[i] = i;
    }
    std::shuffle(sequence.begin(), sequence.end(), std::default_random_engine(threadNo));
    for (std::uint16_t a : sequence) {
        for (std::uint16_t b : sequence) {
            for (std::uint16_t c : sequence) {
                Key key(a, b, c);
                auto insertResult = map.insert(key, calculate);
                if (insertResult.second) {
                    performedInsertions[threadNo]++;
                    auto findResult = map.find(key);
                    if (findResult == map.end()) {
                        failTest("Could not find the inserted entry immediately after insertion.");
                    }
                    const Value& insertedValue = insertResult.first->second;
                    const Value& foundValue = findResult->second;
                    Value calculatedValue = calculate(key);
                    if (insertedValue != foundValue || insertedValue != calculatedValue) {
                        failTest("Inconsistent values after insertion.");
                    }
                }
            }
        }
    }
}

void checkConsistency(const FcmmType& map) {
    for (std::uint16_t a = 0; a <= KEY_FIELD_MAX_VALUE; a++) {
        for (std::uint16_t b = 0; b <= KEY_FIELD_MAX_VALUE; b++) {
            for (std::uint16_t c = 0; c <= KEY_FIELD_MAX_VALUE; c++) {
                Key key(a, b, c);
                auto findResult = map.find(key);
                if (findResult == map.end()) {
                    failTest("Could not find a key.");
                }
                const Value& actualValue = findResult->second;
                Value expectedValue = calculate(key);
                if (actualValue != expectedValue) {
                    failTest("Inconsistent value found.");
                }
            }
        }
    }
}

void performTest(bool verbose = false) {


    // Create and populate the map

    FcmmType map;

    if (!map.empty()) {
        failTest("Map should be empty.");
    }

    std::array<int, NUM_THREADS> performedInsertions;
    std::fill_n(performedInsertions.begin(), performedInsertions.size(), 0);

    runThreads(NUM_THREADS, threadFunction, std::ref(map), std::ref(performedInsertions));


    // Check number of insertions

    std::size_t performedInsertionsTotal = 0;

    for (int threadNo = 0; threadNo < NUM_THREADS; threadNo++) {
        performedInsertionsTotal += performedInsertions[threadNo];
        if (verbose) {
            std::cout << "Thread no: " << threadNo << ", "
                    << "performed insertions: " << performedInsertions[threadNo]
                    << std::endl;
        }
    }

    const std::size_t duplicates = performedInsertionsTotal - EXPECTED_NUM_ENTRIES;

    if (performedInsertionsTotal >= EXPECTED_NUM_ENTRIES) {
        if (verbose) {
            const float duplicatesPercent = 100.0f * duplicates / performedInsertionsTotal;
            std::cout << "Minimum total insertions: " << EXPECTED_NUM_ENTRIES << ", "
                    << "actual: " << performedInsertionsTotal
                    << std::endl
                    << "The map contains " << duplicates << " duplicates (" << duplicatesPercent << "%)"
                    << std::endl;
        }
    } else {
        failTest("Minimum total insertions: " + std::to_string(EXPECTED_NUM_ENTRIES) + ", "
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

    std::size_t counter = 0;
    for (FcmmType::const_iterator it = map.begin(); it != map.end(); ++it) {
        const FcmmType::Entry& entry = *it;
        auto findResult = map.find(entry.first);
        if (findResult == map.end()) {
            failTest("Inconsistent iterator behaviour [1]");
        }
        if (entry.second != findResult->second) {
            failTest("Inconsistent iterator behaviour [2]");
        }
        counter++;
    }

    if (counter != performedInsertionsTotal) {
        failTest("Inconsistent iterator behaviour [3] (counter: " + std::to_string(counter) + ")");
    }

    std::cout << "OK." << std::endl;


    // Check clone() and filter()

    {

        std::cout << "Checking clone(): " << std::flush;

        std::unique_ptr<FcmmType> mapClone(map.clone());

        if (mapClone->size() != performedInsertionsTotal - duplicates) { // clone() will remove duplicates
            failTest("Inconsistent clone() behaviour [1]");
        }

        for (FcmmType::const_iterator it = mapClone->begin(); it != mapClone->end(); it++) {
            const FcmmType::Entry& entry = *it;
            if (entry.second != map.at(entry.first)) {
                failTest("Inconsistent clone() behaviour [2]");
            }
        }

        checkConsistency(*mapClone);

        std::cout << "OK." << std::endl;

    }

    {

        std::cout << "Checking filter(): " << std::flush;

        auto filterFunction = [](const FcmmType::Entry& entry) {
            return entry.first.a == 0;
        };

        std::unique_ptr<FcmmType> filteredMap(map.filter(filterFunction));

        if (filteredMap->getNumEntries() != (KEY_FIELD_MAX_VALUE+1) * (KEY_FIELD_MAX_VALUE+1)) {
            failTest("Inconsistent filter() behaviour [1]");
        }

        for (const auto& entry : *filteredMap) {
            if (entry.first.a != 0) {
                failTest("Inconsistent filter() behaviour [2]");
            }
            if (entry.second != map[entry.first]) {
                failTest("Inconsistent filter() behaviour [3]");
            }
        }

        for (std::uint16_t a = 1; a <= KEY_FIELD_MAX_VALUE; a++) {
            for (std::uint16_t b = 0; b <= KEY_FIELD_MAX_VALUE; b++) {
                for (std::uint16_t c = 0; c <= KEY_FIELD_MAX_VALUE; c++) {
                    Key key(a, b, c);
                    auto findResult = filteredMap->find(key);
                    if (findResult != filteredMap->end()) {
                        failTest("Inconsistent filter() behaviour [4]");
                    }
                }
            }
        }

        std::cout << "OK." << std::endl;

    }


}

int main(void) {

    char* verboseEnv = getenv("FCMM_VERBOSE");
    bool verbose = verboseEnv != NULL && atoi(verboseEnv) == 1;

    std::cout
            << "Correctness test for Fast Concurrent Memoization Map (fcmm)" << std::endl << std::endl
            << "The test will be executed " << TEST_RUNS << " times and may require quite a long time. " << std::endl
            << "Please be patient, this is not a benchmark." << std::endl << std::endl;

    for (int i = 1; i <= TEST_RUNS; i++) {
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Run " << i << " of " << TEST_RUNS << std::endl;
        try {
            performTest(verbose);
        } catch (std::exception& e) {
            failTest("Exception: " + std::string(e.what()));
        }
    }

    std::cout << "TEST SUCCEEDED" << std::endl;

    return EXIT_SUCCESS;

}
