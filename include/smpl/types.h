////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_TYPES_H
#define SMPL_TYPES_H

// standard includes
#include <functional>
#include <unordered_map>
#include <vector>

// system includes
#include <boost/functional/hash.hpp>

namespace smpl {

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

// helper struct to compute a hash value for a pointer using the hash value of
// the object it points to
template <typename T>
struct PointerValueHash
{
    typedef T* argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type s) const { return std::hash<T>()(*s); }
};

// helper struct to test for equality between two pointers by testing for
// equality between the objects they point to
template <typename T>
struct PointerValueEqual
{
    typedef T* argument_type;
    bool operator()(argument_type a, argument_type b) const { return *a == *b; }
};

template <class T, class Allocator = std::allocator<T>>
struct VectorHash
{
    using argument_type = std::vector<T, Allocator>;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type
    {
        auto seed = result_type(0);
        boost::hash_combine(seed, boost::hash_range(begin(s), end(s)));
        return seed;
    }
};

#if 1
using RobotState = std::vector<double>;
#else
// This class is eventually meant to replace the above typedef to add type
// safety to RobotState usages; currently, it is used as a compile-time
// mechanism to ensure for the time being that the RobotState identifier is used
// in all appropriate contexts.
class RobotState : public std::vector<double>
{
public:

    typedef std::vector<double> Base;
    typedef Base::value_type value_type;
    typedef Base::allocator_type allocator_type;
    typedef Base::size_type size_type;
    typedef Base::difference_type difference_type;
    typedef Base::reference reference;
    typedef Base::const_reference const_reference;
    typedef Base::pointer pointer;
    typedef Base::const_pointer const_pointer;
    typedef Base::iterator iterator;
    typedef Base::const_iterator const_iterator;
    typedef Base::reverse_iterator reverse_iterator;
    typedef Base::const_reverse_iterator const_reverse_iterator;

    explicit RobotState(const allocator_type& alloc = allocator_type()) :
        Base(alloc) { }

    RobotState(
        size_type count,
        const double& value = double(),
        const allocator_type& alloc = allocator_type())
    :
        Base(count, value, alloc)
    { }

    explicit RobotState(size_type count) : Base(count) { }

    template <class InputIt>
    RobotState(
        InputIt first,
        InputIt last,
        const allocator_type& alloc = allocator_type())
    :
        Base(first, last, alloc)
    { }

    RobotState(const RobotState& other) : Base(other) { }
    RobotState(const Base& other) : Base(other) { }

    RobotState(const RobotState& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }
    RobotState(const Base& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }

    RobotState(RobotState&& other) : Base(other) { }
    RobotState(Base&& other) : Base(other) { }

    RobotState(RobotState&& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }
    RobotState(Base&& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }

    RobotState(
            std::initializer_list<double> init,
            const allocator_type& alloc = allocator_type())
    :
        Base(init, alloc)
    { }

    RobotState& operator=(const RobotState& other) { Base::operator=(other); return *this; }
    RobotState& operator=(const Base& other) { Base::operator=(other); return *this; }
    RobotState& operator=(RobotState&& other) { Base::operator=(other); return *this; }
    RobotState& operator=(Base&& other) { Base::operator=(other); return *this; }
    RobotState& operator=(std::initializer_list<double> ilist) { Base::operator=(ilist); return *this; }
};
#endif

using Action = std::vector<RobotState>;

} // namespace smpl

#endif
