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

#include <fcmm.hpp>

#include <iostream>

struct Key {
    int i;
    Key() : i(0) {}
    explicit Key(int i) : i(0) {}
    bool operator==(const Key& other) const {
        return i == other.i;
    }
};

struct Value {
    int i;
    Value() : i(0) {}
    explicit Value(int i) : i(0) {}
    bool operator==(const Value& other) const {
        return i == other.i;
    }
};

struct MovableKey {
    int i;
    MovableKey() : i(0) {}
    explicit MovableKey(int i) : i(i) {}
    MovableKey(const MovableKey&) = delete;
    MovableKey(MovableKey&& other) : i(other.i) {}
    MovableKey& operator=(const MovableKey&) = delete;
    MovableKey& operator=(MovableKey&& other) {
        i = other.i;
        return *this;
    }
    bool operator==(const MovableKey& other) const {
        return i == other.i;
    }
};

struct MovableValue {
    int i;
    MovableValue() : i(0) {}
    explicit MovableValue(int i) : i(i) {}
    MovableValue(const MovableValue&) = delete;
    MovableValue(MovableValue&& other) : i(other.i) {}
    MovableValue& operator=(const MovableValue&) = delete;
    MovableValue& operator=(MovableValue&& other) {
        i = other.i;
        return *this;
    }
    bool operator==(const MovableValue& other) const {
        return i == other.i;
    }
};

template<typename K>
struct KeyHash1 {
    std::size_t operator()(const K& k) const {
        return std::hash<int>()(k.i);
    }
};

template<typename K>
struct KeyHash2 {
    std::size_t operator()(const K& k) const {
        return std::hash<int>()(~k.i);
    }
};

template <typename M>
void test(M& fcmm) {
    
    typedef typename M::key_type K;
    typedef typename M::mapped_type V;
    
    K k1(1);
    V v1(2);
    fcmm.emplace(std::move(k1), std::move(v1));
    
    K k2(2);
    V v2(4);
    auto entry = std::make_pair(std::move(k2), std::move(v2));
    fcmm.insert(std::move(entry));
    
    K k3(3);
    V v3(8);
    fcmm.insert(std::move(k3), [&v3](const K&){ return std::move(v3); });
    
}

int main(void) {

    // If this test does not compile, please send me a complete report.
    
    // Only key is movable
    fcmm::Fcmm<MovableKey, Value, KeyHash1<MovableKey>, KeyHash2<MovableKey> > fcmm1;
    test(fcmm1);
    
    // Only value is movable
    fcmm::Fcmm<Key, MovableValue, KeyHash1<Key>, KeyHash2<Key> > fcmm2;
    test(fcmm2);
    
    // Both key and value are movable
    fcmm::Fcmm<MovableKey, MovableValue, KeyHash1<MovableKey>, KeyHash2<MovableKey> > fcmm3;
    test(fcmm3);
    
    return 0;
    
}
