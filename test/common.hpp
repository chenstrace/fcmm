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

#ifndef FCMM_TEST_COMMON_H_
#define FCMM_TEST_COMMON_H_

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <iostream>

struct Key {
    std::uint16_t a;
    std::uint16_t b;
    std::uint16_t c;
    Key() {
    }
    Key(std::uint16_t a, std::uint16_t b, std::uint16_t c) : a(a), b(b), c(c) {
    }
    bool operator==(const Key& other) const {
        return other.a == a && other.b == b && other.c == c;
    }
};

struct KeyHash1 {
    std::size_t operator()(const Key& key) const {
        const std::size_t prime = 2166136261;
        std::size_t hash = 2147483647;
        hash = (hash * prime) ^ key.a;
        hash = (hash * prime) ^ key.b;
        hash = (hash * prime) ^ key.c;
        return hash;
    }
};

struct KeyHash2 {
    std::size_t operator()(const Key& key) const {
        std::size_t hash = 0;
        hash = key.a + (hash << 6) + (hash << 16) - hash;
        hash = key.b + (hash << 6) + (hash << 16) - hash;
        hash = key.c + (hash << 6) + (hash << 16) - hash;
        return hash;
    }
};

struct Value {
    std::int32_t u;
    std::int32_t v;
    std::int32_t w;
    bool operator==(const Value& other) const {
        return other.u == u && other.v == v && other.w == w;
    }
    bool operator!=(const Value& other) const {
        return !operator==(other);
    }
};

static const int CALCULATE_ITERATIONS = 15;

Value calculate(const Key& k) {

    Value value;
    value.u = value.v = value.w = 0;

    // Just some useless computation
    for (int i = 1; i <= CALCULATE_ITERATIONS; i++) {
        value.u += ((i+1)*k.a + (i+8)*k.b + (i+14)*k.c) / i;
    }
    for (int i = 1; i <= CALCULATE_ITERATIONS; i++) {
        value.v += ((i+4)*k.a + (i+2)*k.b + (i+6)*k.c) / (i+1);
    }
    for (int i = 1; i <= CALCULATE_ITERATIONS; i++) {
        value.w += ((i+2)*k.a + (i+6)*k.b + (i+7)*k.c) / (i+2);
    }

    return value;

}

bool checkEntryConsistency(const Key& key, const Value& actualValue, const Value& expectedValue) {
    if (actualValue.u != expectedValue.u || actualValue.v != expectedValue.v || actualValue.w != expectedValue.w) {
        return false;
    }
    return true;
}

template<typename FcmmType>
void printStats(const FcmmType& map) {
    std::cout << "Statistics for the fcmm instance: " << std::endl;
    fcmm::Stats stats = map.getStats();
    std::cout << "Number of entries: " << stats.numEntries << std::endl;
    std::cout << "Number of submaps: " << stats.numSubmaps << std::endl;
    for (std::size_t i = 0; i < stats.numSubmaps; i++) {
        std::cout << "Submap #" << i << std::endl;
        std::cout << "\tCapacity: " << stats.submapsStats[i].capacity << std::endl;
        std::cout << "\tNumber of valid buckets: " << stats.submapsStats[i].numValidBuckets << std::endl;
        std::cout << "\tLoad factor: " << stats.submapsStats[i].loadFactor << std::endl;
    }
}

typedef std::chrono::time_point<std::chrono::system_clock> timestamp_t;

timestamp_t now() {
    return std::chrono::system_clock::now();
}

int elapsedMillis(timestamp_t start, timestamp_t end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
}

template<typename Function, typename... Arguments>
int benchmark(Function function, Arguments... args) {

    timestamp_t start = now();
    function(args...);
    timestamp_t end = now();
    int elapsedMs = elapsedMillis(start, end);

    return elapsedMs;

}

template<typename ThreadFunction, typename... Args>
void runThreads(int numThreads, ThreadFunction threadFunction, Args&&... threadFunctionArgs) {

    std::vector<std::thread> threads;

    for (int threadNo = 0; threadNo < numThreads; threadNo++) {
        std::thread thread(threadFunction, threadNo, std::forward<Args>(threadFunctionArgs)...);
        threads.push_back(std::move(thread));
    }

    for (std::thread& thread : threads) {
        thread.join();
    }

}

#endif // FCMM_TEST_COMMON_H_
