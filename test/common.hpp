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

#ifndef FCMM_TEST_COMMON_H_
#define FCMM_TEST_COMMON_H_

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

static const std::string REPORT_RECIPIENT = "Giacomo Drago <giacomo@giacomodrago.com>";
static const int CALCULATE_ITERATIONS = 15;

struct Key {
    std::uint16_t a;
    std::uint16_t b;
    std::uint16_t c;
    Key() : a(0), b(0), c(0) {
    }
    Key(std::uint16_t a, std::uint16_t b, std::uint16_t c) : a(a), b(b), c(c) {
    }
    bool operator==(const Key& other) const {
        return other.a == a && other.b == b && other.c == c;
    }
    bool operator!=(const Key& other) const {
        return !operator==(other);
    }
    std::string toString() const {
        return "key [" + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) + "]";
    }
};

struct KeyHash1 {
    std::size_t operator()(const Key& key) const {
        const std::size_t prime = 16777619;
        std::size_t hash = 2166136261;
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
    std::string toString() const {
        return "value [" + std::to_string(u) + " " + std::to_string(v) + " " + std::to_string(w) + "]";
    }
};

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

template<typename FcmmType>
void printStats(const FcmmType& map) {
    std::cout << "Statistics for the fcmm instance: " << std::endl;
    const fcmm::Stats stats = map.getStats();
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

long long elapsedMillis(timestamp_t start, timestamp_t end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
}

template<typename Function, typename... Args>
long long benchmark(Function function, Args&&... args) {

    const timestamp_t start = now();
    function(std::forward<Args>(args)...);
    const timestamp_t end = now();
    const long long elapsedMs = elapsedMillis(start, end);

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

void fail(const std::string& message) {
    std::cerr << std::endl << message << std::endl
              << "TEST FAILED" << std::endl
              << "Please report full test output along with your test environment" << std::endl
              << "to " << REPORT_RECIPIENT << std::endl;
    exit(EXIT_FAILURE);
}

void assert(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

#endif // FCMM_TEST_COMMON_H_
