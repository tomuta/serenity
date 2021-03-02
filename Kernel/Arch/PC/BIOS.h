/*
 * Copyright (c) 2020, Andreas Kling <kling@serenityos.org>
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

#include <AK/Types.h>
#include <Kernel/VM/MappedROM.h>

namespace Kernel {

MappedROM map_bios();
MappedROM map_ebda();

class BIOSEmulator;

struct BIOSEmulatorMemory {
    FlatPtr begin;
    FlatPtr end;
    u8* mapped_base;

    enum class Type {
        Code,
        DataReadOnly,
        DataReadWrite
    };
    Type type;

    explicit BIOSEmulatorMemory(FlatPtr begin, FlatPtr end, u8* mapped_base, Type type)
        : begin(begin)
        , end(end)
        , mapped_base(mapped_base)
        , type(type)
    {
    }
    explicit BIOSEmulatorMemory(const MappedROM& rom, Type type)
        : begin(rom.paddr.get())
        , end(rom.paddr.get() + rom.size)
        , mapped_base(rom.region->vaddr().as_ptr())
        , type(type)
    {
    }

    ALWAYS_INLINE bool contains(FlatPtr ptr) const
    {
        if (ptr < begin)
            return false;
        return ptr < end;
    }

    template<typename T>
    ALWAYS_INLINE void write(FlatPtr offset, T value)
    {
        VERIFY(begin + offset <= end);
        *reinterpret_cast<volatile T*>(mapped_base + offset) = value;
    }
    template<typename T>
    ALWAYS_INLINE T read(FlatPtr offset) const
    {
        VERIFY(begin + offset <= end);
        return *reinterpret_cast<volatile T*>(mapped_base + offset);
    }
};

class BIOS {
public:
    BIOS();
    ~BIOS();
    void call(u8 interrupt_number, RegisterState& regs);
private:
    MappedROM m_bios_rom;
    OwnPtr<Region> m_bios_ivt;
    BIOSEmulatorMemory m_bios;
    BIOSEmulatorMemory m_ivt;
    BIOSEmulator* m_emulator;
};


}
