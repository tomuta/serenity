/*
 * Copyright (c) 2020, The SerenityOS developers.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <AK/kmalloc.h>

namespace AK {

template<typename Alloc>
struct AllocatorTraits {
    typedef Alloc Allocator;

    static void* allocate(Alloc& a, size_t size)
    {
        return a.allocate(size);
    }
    static void deallocate(Alloc& a, void* ptr)
    {
        a.deallocate(ptr);
    }
    template<typename T, typename... Args>
    static T* construct(Alloc& a, Args&&... args)
    {
        T* ptr = (T*)allocate(a, sizeof(T));
        if (ptr)
            ::new(ptr) T(forward<Args>(args)...);
        return ptr;
    }
    template<typename T>
    static void destroy(Alloc& a, T* ptr)
    {
        if (!ptr)
            return;
        ptr->~T();
        deallocate(a, ptr);
    }
    template<typename T, typename... Args>
    static T* construct_many(Alloc& a, size_t count, Args&&... args)
    {
        T* ptr = (T*)allocate(a, count * sizeof(T));
        if (ptr) {
            for (size_t i = 0; i < count; i++)
                ::new(&ptr[i]) T(forward<Args>(args)...);
        }
        return ptr;
    }
    template<typename T>
    static void destroy_many(Alloc& a, T* ptr, size_t count)
    {
        if (!ptr)
            return;
        for (size_t i = 0; i < count; i++)
            ptr[i].~T();
        deallocate(a, ptr);
    }
};

struct Allocator {
    void* allocate(size_t size)
    {
        return kmalloc(size);
    }
    void deallocate(void* ptr)
    {
        kfree(ptr);
    }
};

}

using AK::Allocator;
