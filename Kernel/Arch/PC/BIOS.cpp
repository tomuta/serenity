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

#include <AK/StringView.h>
#include <Kernel/Arch/PC/BIOS.h>
#include <Kernel/VM/MemoryManager.h>
#include <Kernel/VM/TypedMapping.h>
#include <LibX86/Instruction.h>
#include <LibX86/Interpreter.h>

#define MEMORY_DEBUG 1

namespace Kernel {

MappedROM map_bios()
{
    MappedROM mapping;
    mapping.size = 128 * KiB;
    mapping.paddr = PhysicalAddress(0xe0000);
    mapping.region = MM.allocate_kernel_region(mapping.paddr, page_round_up(mapping.size), {}, Region::Access::Read);
    return mapping;
}

MappedROM map_ebda()
{
    auto ebda_segment_ptr = map_typed<u16>(PhysicalAddress(0x40e));
    auto ebda_length_ptr = map_typed<u16>(PhysicalAddress(0x413));

    PhysicalAddress ebda_paddr(*ebda_segment_ptr << 4);
    size_t ebda_size = *ebda_length_ptr;

    MappedROM mapping;
    mapping.region = MM.allocate_kernel_region(ebda_paddr.page_base(), page_round_up(ebda_size), {}, Region::Access::Read);
    mapping.offset = ebda_paddr.offset_in_page();
    mapping.size = ebda_size;
    mapping.paddr = ebda_paddr;
    return mapping;
}

union PartAddressableRegister {
    struct {
        u32 full_u32;
    };
    struct {
        u16 low_u16;
        u16 high_u16;
    };
    struct {
        u8 low_u8;
        u8 high_u8;
        u16 also_high_u16;
    };
};

class SoftCPU final
    : public X86::Interpreter
    , public X86::InstructionStream {
public:
    template<typename T>
    class Value {
    public:
        Value(T value)
            : m_value(value)
        {
        }
        template<typename U>
        auto operator<=>(U value) const { return m_value <=> value; }
        template<typename U>
        auto operator<=>(const Value<U>& other) const { return m_value <=> other.m_value; }
        T value() const { return m_value; }
        operator T() const { return m_value; }
    private:
        T m_value;
    };
    using ValueType8 = Value<u8>;
    using ValueType16 = Value<u16>;
    using ValueType32 = Value<u32>;
    using ValueType64 = Value<u64>;

    SoftCPU(BIOSEmulator& emulator)
        : m_emulator(emulator)
    {
    }
    void dump() const;

    u32 eip() const { return m_eip; }
    void set_eip(u32 eip)
    {
        m_eip = eip;
    }

    struct Flags {
        enum Flag {
            CF = 0x0001,
            PF = 0x0004,
            AF = 0x0010,
            ZF = 0x0040,
            SF = 0x0080,
            TF = 0x0100,
            IF = 0x0200,
            DF = 0x0400,
            OF = 0x0800,
        };
    };

    void push32(u32);
    u32 pop32();

    void push16(u16);
    u16 pop16();

    u16 segment(X86::SegmentRegister seg) const { return m_segment[(int)seg]; }
    u16& segment(X86::SegmentRegister seg) { return m_segment[(int)seg]; }

    u8& gpr8(X86::RegisterIndex8 reg)
    {
        switch (reg) {
        case X86::RegisterAL:
            return m_gpr[X86::RegisterEAX].low_u8;
        case X86::RegisterAH:
            return m_gpr[X86::RegisterEAX].high_u8;
        case X86::RegisterBL:
            return m_gpr[X86::RegisterEBX].low_u8;
        case X86::RegisterBH:
            return m_gpr[X86::RegisterEBX].high_u8;
        case X86::RegisterCL:
            return m_gpr[X86::RegisterECX].low_u8;
        case X86::RegisterCH:
            return m_gpr[X86::RegisterECX].high_u8;
        case X86::RegisterDL:
            return m_gpr[X86::RegisterEDX].low_u8;
        case X86::RegisterDH:
            return m_gpr[X86::RegisterEDX].high_u8;
        }
        VERIFY_NOT_REACHED();
    }

    Value<u8> const_gpr8(X86::RegisterIndex8 reg) const
    {
        switch (reg) {
        case X86::RegisterAL:
            return m_gpr[X86::RegisterEAX].low_u8;
        case X86::RegisterAH:
            return m_gpr[X86::RegisterEAX].high_u8;
        case X86::RegisterBL:
            return m_gpr[X86::RegisterEBX].low_u8;
        case X86::RegisterBH:
            return m_gpr[X86::RegisterEBX].high_u8;
        case X86::RegisterCL:
            return m_gpr[X86::RegisterECX].low_u8;
        case X86::RegisterCH:
            return m_gpr[X86::RegisterECX].high_u8;
        case X86::RegisterDL:
            return m_gpr[X86::RegisterEDX].low_u8;
        case X86::RegisterDH:
            return m_gpr[X86::RegisterEDX].high_u8;
        }
        VERIFY_NOT_REACHED();
    }

    Value<u16> const_gpr16(X86::RegisterIndex16 reg) const
    {
        return m_gpr[reg].low_u16;
    }

    u16& gpr16(X86::RegisterIndex16 reg)
    {
        return m_gpr[reg].low_u16;
    }

    Value<u32> const_gpr32(X86::RegisterIndex32 reg) const
    {
        return m_gpr[reg].full_u32;
    }

    u32& gpr32(X86::RegisterIndex32 reg)
    {
        return m_gpr[reg].full_u32;
    }

    template<typename T>
    Value<T> const_gpr(unsigned register_index) const
    {
        if constexpr (sizeof(T) == 1)
            return const_gpr8((X86::RegisterIndex8)register_index);
        if constexpr (sizeof(T) == 2)
            return const_gpr16((X86::RegisterIndex16)register_index);
        if constexpr (sizeof(T) == 4)
            return const_gpr32((X86::RegisterIndex32)register_index);
    }

    template<typename T>
    T& gpr(unsigned register_index)
    {
        if constexpr (sizeof(T) == 1)
            return gpr8((X86::RegisterIndex8)register_index);
        if constexpr (sizeof(T) == 2)
            return gpr16((X86::RegisterIndex16)register_index);
        if constexpr (sizeof(T) == 4)
            return gpr32((X86::RegisterIndex32)register_index);
    }

    u32 source_index(bool a32) const
    {
        if (a32)
            return esi();
        return si();
    }

    u32 destination_index(bool a32) const
    {
        if (a32)
            return edi();
        return di();
    }

    u32 loop_index(bool a32) const
    {
        if (a32)
            return ecx();
        return cx();
    }

    bool decrement_loop_index(bool a32)
    {
        if (a32) {
            set_ecx(ecx() - 1);
            return ecx() == 0;
        }
        set_cx(cx() - 1);
        return cx() == 0;
    }

    ALWAYS_INLINE void step_source_index(bool a32, u32 step)
    {
        if (a32) {
            if (df())
                set_esi(esi() - step);
            else
                set_esi(esi() + step);
        } else {
            if (df())
                set_si(si()- step);
            else
                set_si(si() + step);
        }
    }

    ALWAYS_INLINE void step_destination_index(bool a32, u32 step)
    {
        if (a32) {
            if (df())
                set_edi(edi() - step);
            else
                set_edi(edi() + step);
        } else {
            if (df())
                set_di(di() - step);
            else
                set_di(di() + step);
        }
    }

    u32 eflags() const { return m_eflags; }
    void set_eflags(u32 eflags)
    {
        m_eflags = eflags;
    }

    Value<u32> eax() const { return const_gpr32(X86::RegisterEAX); }
    Value<u32> ebx() const { return const_gpr32(X86::RegisterEBX); }
    Value<u32> ecx() const { return const_gpr32(X86::RegisterECX); }
    Value<u32> edx() const { return const_gpr32(X86::RegisterEDX); }
    Value<u32> esp() const { return const_gpr32(X86::RegisterESP); }
    Value<u32> ebp() const { return const_gpr32(X86::RegisterEBP); }
    Value<u32> esi() const { return const_gpr32(X86::RegisterESI); }
    Value<u32> edi() const { return const_gpr32(X86::RegisterEDI); }

    Value<u16> ax() const { return const_gpr16(X86::RegisterAX); }
    Value<u16> bx() const { return const_gpr16(X86::RegisterBX); }
    Value<u16> cx() const { return const_gpr16(X86::RegisterCX); }
    Value<u16> dx() const { return const_gpr16(X86::RegisterDX); }
    Value<u16> sp() const { return const_gpr16(X86::RegisterSP); }
    Value<u16> bp() const { return const_gpr16(X86::RegisterBP); }
    Value<u16> si() const { return const_gpr16(X86::RegisterSI); }
    Value<u16> di() const { return const_gpr16(X86::RegisterDI); }

    Value<u8> al() const { return const_gpr8(X86::RegisterAL); }
    Value<u8> ah() const { return const_gpr8(X86::RegisterAH); }
    Value<u8> bl() const { return const_gpr8(X86::RegisterBL); }
    Value<u8> bh() const { return const_gpr8(X86::RegisterBH); }
    Value<u8> cl() const { return const_gpr8(X86::RegisterCL); }
    Value<u8> ch() const { return const_gpr8(X86::RegisterCH); }
    Value<u8> dl() const { return const_gpr8(X86::RegisterDL); }
    Value<u8> dh() const { return const_gpr8(X86::RegisterDH); }

    void set_eax(u32 value) { gpr32(X86::RegisterEAX) = value; }
    void set_ebx(u32 value) { gpr32(X86::RegisterEBX) = value; }
    void set_ecx(u32 value) { gpr32(X86::RegisterECX) = value; }
    void set_edx(u32 value) { gpr32(X86::RegisterEDX) = value; }
    void set_esp(u32 value) { gpr32(X86::RegisterESP) = value; }
    void set_ebp(u32 value) { gpr32(X86::RegisterEBP) = value; }
    void set_esi(u32 value) { gpr32(X86::RegisterESI) = value; }
    void set_edi(u32 value) { gpr32(X86::RegisterEDI) = value; }

    void set_ax(u16 value) { gpr16(X86::RegisterAX) = value; }
    void set_bx(u16 value) { gpr16(X86::RegisterBX) = value; }
    void set_cx(u16 value) { gpr16(X86::RegisterCX) = value; }
    void set_dx(u16 value) { gpr16(X86::RegisterDX) = value; }
    void set_sp(u16 value) { gpr16(X86::RegisterSP) = value; }
    void set_bp(u16 value) { gpr16(X86::RegisterBP) = value; }
    void set_si(u16 value) { gpr16(X86::RegisterSI) = value; }
    void set_di(u16 value) { gpr16(X86::RegisterDI) = value; }

    void set_al(u8 value) { gpr8(X86::RegisterAL) = value; }
    void set_ah(u8 value) { gpr8(X86::RegisterAH) = value; }
    void set_bl(u8 value) { gpr8(X86::RegisterBL) = value; }
    void set_bh(u8 value) { gpr8(X86::RegisterBH) = value; }
    void set_cl(u8 value) { gpr8(X86::RegisterCL) = value; }
    void set_ch(u8 value) { gpr8(X86::RegisterCH) = value; }
    void set_dl(u8 value) { gpr8(X86::RegisterDL) = value; }
    void set_dh(u8 value) { gpr8(X86::RegisterDH) = value; }

    bool of() const { return m_eflags & Flags::OF; }
    bool sf() const { return m_eflags & Flags::SF; }
    bool zf() const { return m_eflags & Flags::ZF; }
    bool af() const { return m_eflags & Flags::AF; }
    bool pf() const { return m_eflags & Flags::PF; }
    bool cf() const { return m_eflags & Flags::CF; }
    bool df() const { return m_eflags & Flags::DF; }

    void set_flag(Flags::Flag flag, bool value)
    {
        if (value)
            m_eflags |= flag;
        else
            m_eflags &= ~flag;
    }

    void set_of(bool value) { set_flag(Flags::OF, value); }
    void set_sf(bool value) { set_flag(Flags::SF, value); }
    void set_zf(bool value) { set_flag(Flags::ZF, value); }
    void set_af(bool value) { set_flag(Flags::AF, value); }
    void set_pf(bool value) { set_flag(Flags::PF, value); }
    void set_cf(bool value) { set_flag(Flags::CF, value); }
    void set_df(bool value) { set_flag(Flags::DF, value); }

    void set_flags_with_mask(u32 new_flags, u32 mask)
    {
        m_eflags &= ~mask;
        m_eflags |= new_flags & mask;
    }

    void set_flags_oszapc(u32 new_flags)
    {
        set_flags_with_mask(new_flags, Flags::OF | Flags::SF | Flags::ZF | Flags::AF | Flags::PF | Flags::CF);
    }

    void set_flags_oszap(u32 new_flags)
    {
        set_flags_with_mask(new_flags, Flags::OF | Flags::SF | Flags::ZF | Flags::AF | Flags::PF);
    }

    void set_flags_oszpc(u32 new_flags)
    {
        set_flags_with_mask(new_flags, Flags::OF | Flags::SF | Flags::ZF | Flags::PF | Flags::CF);
    }

    void set_flags_oc(u32 new_flags)
    {
        set_flags_with_mask(new_flags, Flags::OF | Flags::CF);
    }

    u16 cs() const { return m_segment[(int)X86::SegmentRegister::CS]; }
    u16 ds() const { return m_segment[(int)X86::SegmentRegister::DS]; }
    u16 es() const { return m_segment[(int)X86::SegmentRegister::ES]; }
    u16 ss() const { return m_segment[(int)X86::SegmentRegister::SS]; }

    FlatPtr translate_address(X86::LogicalAddress address);
    u8 read_memory8(X86::LogicalAddress);
    u16 read_memory16(X86::LogicalAddress);
    u32 read_memory32(X86::LogicalAddress);
    u64 read_memory64(X86::LogicalAddress);

    template<typename T>
    T read_memory(X86::LogicalAddress address)
    {
        if constexpr (sizeof(T) == 1)
            return read_memory8(address);
        if constexpr (sizeof(T) == 2)
            return read_memory16(address);
        if constexpr (sizeof(T) == 4)
            return read_memory32(address);
    }

    void write_memory8(X86::LogicalAddress, u8);
    void write_memory16(X86::LogicalAddress, u16);
    void write_memory32(X86::LogicalAddress, u32);
    void write_memory64(X86::LogicalAddress, u64);

    template<typename T>
    void write_memory(X86::LogicalAddress address, T data)
    {
        if constexpr (sizeof(T) == 1)
            return write_memory8(address, data);
        if constexpr (sizeof(T) == 2)
            return write_memory16(address, data);
        if constexpr (sizeof(T) == 4)
            return write_memory32(address, data);
    }

    bool evaluate_condition(u8 condition) const
    {
        switch (condition) {
        case 0:
            return of(); // O
        case 1:
            return !of(); // NO
        case 2:
            return cf(); // B, C, NAE
        case 3:
            return !cf(); // NB, NC, AE
        case 4:
            return zf(); // E, Z
        case 5:
            return !zf(); // NE, NZ
        case 6:
            return (cf() | zf()); // BE, NA
        case 7:
            return !(cf() | zf()); // NBE, A
        case 8:
            return sf(); // S
        case 9:
            return !sf(); // NS
        case 10:
            return pf(); // P, PE
        case 11:
            return !pf(); // NP, PO
        case 12:
            return sf() ^ of(); // L, NGE
        case 13:
            return !(sf() ^ of()); // NL, GE
        case 14:
            return (sf() ^ of()) | zf(); // LE, NG
        case 15:
            return !((sf() ^ of()) | zf()); // NLE, G
        default:
            VERIFY_NOT_REACHED();
        }
        return 0;
    }

    template<bool check_zf, typename Callback>
    void do_once_or_repeat(const X86::Instruction& insn, Callback);

    // ^X86::InstructionStream
    virtual bool can_read() override { return false; }
    virtual u8 read8() override;
    virtual u16 read16() override;
    virtual u32 read32() override;
    virtual u64 read64() override;

private:
    // ^X86::Interpreter
    virtual void AAA(const X86::Instruction&) override;
    virtual void AAD(const X86::Instruction&) override;
    virtual void AAM(const X86::Instruction&) override;
    virtual void AAS(const X86::Instruction&) override;
    virtual void ADC_AL_imm8(const X86::Instruction&) override;
    virtual void ADC_AX_imm16(const X86::Instruction&) override;
    virtual void ADC_EAX_imm32(const X86::Instruction&) override;
    virtual void ADC_RM16_imm16(const X86::Instruction&) override;
    virtual void ADC_RM16_imm8(const X86::Instruction&) override;
    virtual void ADC_RM16_reg16(const X86::Instruction&) override;
    virtual void ADC_RM32_imm32(const X86::Instruction&) override;
    virtual void ADC_RM32_imm8(const X86::Instruction&) override;
    virtual void ADC_RM32_reg32(const X86::Instruction&) override;
    virtual void ADC_RM8_imm8(const X86::Instruction&) override;
    virtual void ADC_RM8_reg8(const X86::Instruction&) override;
    virtual void ADC_reg16_RM16(const X86::Instruction&) override;
    virtual void ADC_reg32_RM32(const X86::Instruction&) override;
    virtual void ADC_reg8_RM8(const X86::Instruction&) override;
    virtual void ADD_AL_imm8(const X86::Instruction&) override;
    virtual void ADD_AX_imm16(const X86::Instruction&) override;
    virtual void ADD_EAX_imm32(const X86::Instruction&) override;
    virtual void ADD_RM16_imm16(const X86::Instruction&) override;
    virtual void ADD_RM16_imm8(const X86::Instruction&) override;
    virtual void ADD_RM16_reg16(const X86::Instruction&) override;
    virtual void ADD_RM32_imm32(const X86::Instruction&) override;
    virtual void ADD_RM32_imm8(const X86::Instruction&) override;
    virtual void ADD_RM32_reg32(const X86::Instruction&) override;
    virtual void ADD_RM8_imm8(const X86::Instruction&) override;
    virtual void ADD_RM8_reg8(const X86::Instruction&) override;
    virtual void ADD_reg16_RM16(const X86::Instruction&) override;
    virtual void ADD_reg32_RM32(const X86::Instruction&) override;
    virtual void ADD_reg8_RM8(const X86::Instruction&) override;
    virtual void AND_AL_imm8(const X86::Instruction&) override;
    virtual void AND_AX_imm16(const X86::Instruction&) override;
    virtual void AND_EAX_imm32(const X86::Instruction&) override;
    virtual void AND_RM16_imm16(const X86::Instruction&) override;
    virtual void AND_RM16_imm8(const X86::Instruction&) override;
    virtual void AND_RM16_reg16(const X86::Instruction&) override;
    virtual void AND_RM32_imm32(const X86::Instruction&) override;
    virtual void AND_RM32_imm8(const X86::Instruction&) override;
    virtual void AND_RM32_reg32(const X86::Instruction&) override;
    virtual void AND_RM8_imm8(const X86::Instruction&) override;
    virtual void AND_RM8_reg8(const X86::Instruction&) override;
    virtual void AND_reg16_RM16(const X86::Instruction&) override;
    virtual void AND_reg32_RM32(const X86::Instruction&) override;
    virtual void AND_reg8_RM8(const X86::Instruction&) override;
    virtual void ARPL(const X86::Instruction&) override;
    virtual void BOUND(const X86::Instruction&) override;
    virtual void BSF_reg16_RM16(const X86::Instruction&) override;
    virtual void BSF_reg32_RM32(const X86::Instruction&) override;
    virtual void BSR_reg16_RM16(const X86::Instruction&) override;
    virtual void BSR_reg32_RM32(const X86::Instruction&) override;
    virtual void BSWAP_reg32(const X86::Instruction&) override;
    virtual void BTC_RM16_imm8(const X86::Instruction&) override;
    virtual void BTC_RM16_reg16(const X86::Instruction&) override;
    virtual void BTC_RM32_imm8(const X86::Instruction&) override;
    virtual void BTC_RM32_reg32(const X86::Instruction&) override;
    virtual void BTR_RM16_imm8(const X86::Instruction&) override;
    virtual void BTR_RM16_reg16(const X86::Instruction&) override;
    virtual void BTR_RM32_imm8(const X86::Instruction&) override;
    virtual void BTR_RM32_reg32(const X86::Instruction&) override;
    virtual void BTS_RM16_imm8(const X86::Instruction&) override;
    virtual void BTS_RM16_reg16(const X86::Instruction&) override;
    virtual void BTS_RM32_imm8(const X86::Instruction&) override;
    virtual void BTS_RM32_reg32(const X86::Instruction&) override;
    virtual void BT_RM16_imm8(const X86::Instruction&) override;
    virtual void BT_RM16_reg16(const X86::Instruction&) override;
    virtual void BT_RM32_imm8(const X86::Instruction&) override;
    virtual void BT_RM32_reg32(const X86::Instruction&) override;
    virtual void CALL_FAR_mem16(const X86::Instruction&) override;
    virtual void CALL_FAR_mem32(const X86::Instruction&) override;
    virtual void CALL_RM16(const X86::Instruction&) override;
    virtual void CALL_RM32(const X86::Instruction&) override;
    virtual void CALL_imm16(const X86::Instruction&) override;
    virtual void CALL_imm16_imm16(const X86::Instruction&) override;
    virtual void CALL_imm16_imm32(const X86::Instruction&) override;
    virtual void CALL_imm32(const X86::Instruction&) override;
    virtual void CBW(const X86::Instruction&) override;
    virtual void CDQ(const X86::Instruction&) override;
    virtual void CLC(const X86::Instruction&) override;
    virtual void CLD(const X86::Instruction&) override;
    virtual void CLI(const X86::Instruction&) override;
    virtual void CLTS(const X86::Instruction&) override;
    virtual void CMC(const X86::Instruction&) override;
    virtual void CMOVcc_reg16_RM16(const X86::Instruction&) override;
    virtual void CMOVcc_reg32_RM32(const X86::Instruction&) override;
    virtual void CMPSB(const X86::Instruction&) override;
    virtual void CMPSD(const X86::Instruction&) override;
    virtual void CMPSW(const X86::Instruction&) override;
    virtual void CMPXCHG_RM16_reg16(const X86::Instruction&) override;
    virtual void CMPXCHG_RM32_reg32(const X86::Instruction&) override;
    virtual void CMPXCHG_RM8_reg8(const X86::Instruction&) override;
    virtual void CMP_AL_imm8(const X86::Instruction&) override;
    virtual void CMP_AX_imm16(const X86::Instruction&) override;
    virtual void CMP_EAX_imm32(const X86::Instruction&) override;
    virtual void CMP_RM16_imm16(const X86::Instruction&) override;
    virtual void CMP_RM16_imm8(const X86::Instruction&) override;
    virtual void CMP_RM16_reg16(const X86::Instruction&) override;
    virtual void CMP_RM32_imm32(const X86::Instruction&) override;
    virtual void CMP_RM32_imm8(const X86::Instruction&) override;
    virtual void CMP_RM32_reg32(const X86::Instruction&) override;
    virtual void CMP_RM8_imm8(const X86::Instruction&) override;
    virtual void CMP_RM8_reg8(const X86::Instruction&) override;
    virtual void CMP_reg16_RM16(const X86::Instruction&) override;
    virtual void CMP_reg32_RM32(const X86::Instruction&) override;
    virtual void CMP_reg8_RM8(const X86::Instruction&) override;
    virtual void CPUID(const X86::Instruction&) override;
    virtual void CWD(const X86::Instruction&) override;
    virtual void CWDE(const X86::Instruction&) override;
    virtual void DAA(const X86::Instruction&) override;
    virtual void DAS(const X86::Instruction&) override;
    virtual void DEC_RM16(const X86::Instruction&) override;
    virtual void DEC_RM32(const X86::Instruction&) override;
    virtual void DEC_RM8(const X86::Instruction&) override;
    virtual void DEC_reg16(const X86::Instruction&) override;
    virtual void DEC_reg32(const X86::Instruction&) override;
    virtual void DIV_RM16(const X86::Instruction&) override;
    virtual void DIV_RM32(const X86::Instruction&) override;
    virtual void DIV_RM8(const X86::Instruction&) override;
    virtual void ENTER16(const X86::Instruction&) override;
    virtual void ENTER32(const X86::Instruction&) override;
    virtual void ESCAPE(const X86::Instruction&) override;
    virtual void FADD_RM32(const X86::Instruction&) override;
    virtual void FMUL_RM32(const X86::Instruction&) override;
    virtual void FCOM_RM32(const X86::Instruction&) override;
    virtual void FCOMP_RM32(const X86::Instruction&) override;
    virtual void FSUB_RM32(const X86::Instruction&) override;
    virtual void FSUBR_RM32(const X86::Instruction&) override;
    virtual void FDIV_RM32(const X86::Instruction&) override;
    virtual void FDIVR_RM32(const X86::Instruction&) override;
    virtual void FLD_RM32(const X86::Instruction&) override;
    virtual void FXCH(const X86::Instruction&) override;
    virtual void FST_RM32(const X86::Instruction&) override;
    virtual void FNOP(const X86::Instruction&) override;
    virtual void FSTP_RM32(const X86::Instruction&) override;
    virtual void FLDENV(const X86::Instruction&) override;
    virtual void FCHS(const X86::Instruction&) override;
    virtual void FABS(const X86::Instruction&) override;
    virtual void FTST(const X86::Instruction&) override;
    virtual void FXAM(const X86::Instruction&) override;
    virtual void FLDCW(const X86::Instruction&) override;
    virtual void FLD1(const X86::Instruction&) override;
    virtual void FLDL2T(const X86::Instruction&) override;
    virtual void FLDL2E(const X86::Instruction&) override;
    virtual void FLDPI(const X86::Instruction&) override;
    virtual void FLDLG2(const X86::Instruction&) override;
    virtual void FLDLN2(const X86::Instruction&) override;
    virtual void FLDZ(const X86::Instruction&) override;
    virtual void FNSTENV(const X86::Instruction&) override;
    virtual void F2XM1(const X86::Instruction&) override;
    virtual void FYL2X(const X86::Instruction&) override;
    virtual void FPTAN(const X86::Instruction&) override;
    virtual void FPATAN(const X86::Instruction&) override;
    virtual void FXTRACT(const X86::Instruction&) override;
    virtual void FPREM1(const X86::Instruction&) override;
    virtual void FDECSTP(const X86::Instruction&) override;
    virtual void FINCSTP(const X86::Instruction&) override;
    virtual void FNSTCW(const X86::Instruction&) override;
    virtual void FPREM(const X86::Instruction&) override;
    virtual void FYL2XP1(const X86::Instruction&) override;
    virtual void FSQRT(const X86::Instruction&) override;
    virtual void FSINCOS(const X86::Instruction&) override;
    virtual void FRNDINT(const X86::Instruction&) override;
    virtual void FSCALE(const X86::Instruction&) override;
    virtual void FSIN(const X86::Instruction&) override;
    virtual void FCOS(const X86::Instruction&) override;
    virtual void FIADD_RM32(const X86::Instruction&) override;
    virtual void FCMOVB(const X86::Instruction&) override;
    virtual void FIMUL_RM32(const X86::Instruction&) override;
    virtual void FCMOVE(const X86::Instruction&) override;
    virtual void FICOM_RM32(const X86::Instruction&) override;
    virtual void FCMOVBE(const X86::Instruction&) override;
    virtual void FICOMP_RM32(const X86::Instruction&) override;
    virtual void FCMOVU(const X86::Instruction&) override;
    virtual void FISUB_RM32(const X86::Instruction&) override;
    virtual void FISUBR_RM32(const X86::Instruction&) override;
    virtual void FUCOMPP(const X86::Instruction&) override;
    virtual void FIDIV_RM32(const X86::Instruction&) override;
    virtual void FIDIVR_RM32(const X86::Instruction&) override;
    virtual void FILD_RM32(const X86::Instruction&) override;
    virtual void FCMOVNB(const X86::Instruction&) override;
    virtual void FISTTP_RM32(const X86::Instruction&) override;
    virtual void FCMOVNE(const X86::Instruction&) override;
    virtual void FIST_RM32(const X86::Instruction&) override;
    virtual void FCMOVNBE(const X86::Instruction&) override;
    virtual void FISTP_RM32(const X86::Instruction&) override;
    virtual void FCMOVNU(const X86::Instruction&) override;
    virtual void FNENI(const X86::Instruction&) override;
    virtual void FNDISI(const X86::Instruction&) override;
    virtual void FNCLEX(const X86::Instruction&) override;
    virtual void FNINIT(const X86::Instruction&) override;
    virtual void FNSETPM(const X86::Instruction&) override;
    virtual void FLD_RM80(const X86::Instruction&) override;
    virtual void FUCOMI(const X86::Instruction&) override;
    virtual void FCOMI(const X86::Instruction&) override;
    virtual void FSTP_RM80(const X86::Instruction&) override;
    virtual void FADD_RM64(const X86::Instruction&) override;
    virtual void FMUL_RM64(const X86::Instruction&) override;
    virtual void FCOM_RM64(const X86::Instruction&) override;
    virtual void FCOMP_RM64(const X86::Instruction&) override;
    virtual void FSUB_RM64(const X86::Instruction&) override;
    virtual void FSUBR_RM64(const X86::Instruction&) override;
    virtual void FDIV_RM64(const X86::Instruction&) override;
    virtual void FDIVR_RM64(const X86::Instruction&) override;
    virtual void FLD_RM64(const X86::Instruction&) override;
    virtual void FFREE(const X86::Instruction&) override;
    virtual void FISTTP_RM64(const X86::Instruction&) override;
    virtual void FST_RM64(const X86::Instruction&) override;
    virtual void FSTP_RM64(const X86::Instruction&) override;
    virtual void FRSTOR(const X86::Instruction&) override;
    virtual void FUCOM(const X86::Instruction&) override;
    virtual void FUCOMP(const X86::Instruction&) override;
    virtual void FNSAVE(const X86::Instruction&) override;
    virtual void FNSTSW(const X86::Instruction&) override;
    virtual void FIADD_RM16(const X86::Instruction&) override;
    virtual void FADDP(const X86::Instruction&) override;
    virtual void FIMUL_RM16(const X86::Instruction&) override;
    virtual void FMULP(const X86::Instruction&) override;
    virtual void FICOM_RM16(const X86::Instruction&) override;
    virtual void FICOMP_RM16(const X86::Instruction&) override;
    virtual void FCOMPP(const X86::Instruction&) override;
    virtual void FISUB_RM16(const X86::Instruction&) override;
    virtual void FSUBRP(const X86::Instruction&) override;
    virtual void FISUBR_RM16(const X86::Instruction&) override;
    virtual void FSUBP(const X86::Instruction&) override;
    virtual void FIDIV_RM16(const X86::Instruction&) override;
    virtual void FDIVRP(const X86::Instruction&) override;
    virtual void FIDIVR_RM16(const X86::Instruction&) override;
    virtual void FDIVP(const X86::Instruction&) override;
    virtual void FILD_RM16(const X86::Instruction&) override;
    virtual void FFREEP(const X86::Instruction&) override;
    virtual void FISTTP_RM16(const X86::Instruction&) override;
    virtual void FIST_RM16(const X86::Instruction&) override;
    virtual void FISTP_RM16(const X86::Instruction&) override;
    virtual void FBLD_M80(const X86::Instruction&) override;
    virtual void FNSTSW_AX(const X86::Instruction&) override;
    virtual void FILD_RM64(const X86::Instruction&) override;
    virtual void FUCOMIP(const X86::Instruction&) override;
    virtual void FBSTP_M80(const X86::Instruction&) override;
    virtual void FCOMIP(const X86::Instruction&) override;
    virtual void FISTP_RM64(const X86::Instruction&) override;
    virtual void HLT(const X86::Instruction&) override;
    virtual void IDIV_RM16(const X86::Instruction&) override;
    virtual void IDIV_RM32(const X86::Instruction&) override;
    virtual void IDIV_RM8(const X86::Instruction&) override;
    virtual void IMUL_RM16(const X86::Instruction&) override;
    virtual void IMUL_RM32(const X86::Instruction&) override;
    virtual void IMUL_RM8(const X86::Instruction&) override;
    virtual void IMUL_reg16_RM16(const X86::Instruction&) override;
    virtual void IMUL_reg16_RM16_imm16(const X86::Instruction&) override;
    virtual void IMUL_reg16_RM16_imm8(const X86::Instruction&) override;
    virtual void IMUL_reg32_RM32(const X86::Instruction&) override;
    virtual void IMUL_reg32_RM32_imm32(const X86::Instruction&) override;
    virtual void IMUL_reg32_RM32_imm8(const X86::Instruction&) override;
    virtual void INC_RM16(const X86::Instruction&) override;
    virtual void INC_RM32(const X86::Instruction&) override;
    virtual void INC_RM8(const X86::Instruction&) override;
    virtual void INC_reg16(const X86::Instruction&) override;
    virtual void INC_reg32(const X86::Instruction&) override;
    virtual void INSB(const X86::Instruction&) override;
    virtual void INSD(const X86::Instruction&) override;
    virtual void INSW(const X86::Instruction&) override;
    virtual void INT3(const X86::Instruction&) override;
    virtual void INTO(const X86::Instruction&) override;
    virtual void INT_imm8(const X86::Instruction&) override;
    virtual void INVLPG(const X86::Instruction&) override;
    virtual void IN_AL_DX(const X86::Instruction&) override;
    virtual void IN_AL_imm8(const X86::Instruction&) override;
    virtual void IN_AX_DX(const X86::Instruction&) override;
    virtual void IN_AX_imm8(const X86::Instruction&) override;
    virtual void IN_EAX_DX(const X86::Instruction&) override;
    virtual void IN_EAX_imm8(const X86::Instruction&) override;
    virtual void IRET(const X86::Instruction&) override;
    virtual void JCXZ_imm8(const X86::Instruction&) override;
    virtual void JMP_FAR_mem16(const X86::Instruction&) override;
    virtual void JMP_FAR_mem32(const X86::Instruction&) override;
    virtual void JMP_RM16(const X86::Instruction&) override;
    virtual void JMP_RM32(const X86::Instruction&) override;
    virtual void JMP_imm16(const X86::Instruction&) override;
    virtual void JMP_imm16_imm16(const X86::Instruction&) override;
    virtual void JMP_imm16_imm32(const X86::Instruction&) override;
    virtual void JMP_imm32(const X86::Instruction&) override;
    virtual void JMP_short_imm8(const X86::Instruction&) override;
    virtual void Jcc_NEAR_imm(const X86::Instruction&) override;
    virtual void Jcc_imm8(const X86::Instruction&) override;
    virtual void LAHF(const X86::Instruction&) override;
    virtual void LAR_reg16_RM16(const X86::Instruction&) override;
    virtual void LAR_reg32_RM32(const X86::Instruction&) override;
    virtual void LDS_reg16_mem16(const X86::Instruction&) override;
    virtual void LDS_reg32_mem32(const X86::Instruction&) override;
    virtual void LEAVE16(const X86::Instruction&) override;
    virtual void LEAVE32(const X86::Instruction&) override;
    virtual void LEA_reg16_mem16(const X86::Instruction&) override;
    virtual void LEA_reg32_mem32(const X86::Instruction&) override;
    virtual void LES_reg16_mem16(const X86::Instruction&) override;
    virtual void LES_reg32_mem32(const X86::Instruction&) override;
    virtual void LFS_reg16_mem16(const X86::Instruction&) override;
    virtual void LFS_reg32_mem32(const X86::Instruction&) override;
    virtual void LGDT(const X86::Instruction&) override;
    virtual void LGS_reg16_mem16(const X86::Instruction&) override;
    virtual void LGS_reg32_mem32(const X86::Instruction&) override;
    virtual void LIDT(const X86::Instruction&) override;
    virtual void LLDT_RM16(const X86::Instruction&) override;
    virtual void LMSW_RM16(const X86::Instruction&) override;
    virtual void LODSB(const X86::Instruction&) override;
    virtual void LODSD(const X86::Instruction&) override;
    virtual void LODSW(const X86::Instruction&) override;
    virtual void LOOPNZ_imm8(const X86::Instruction&) override;
    virtual void LOOPZ_imm8(const X86::Instruction&) override;
    virtual void LOOP_imm8(const X86::Instruction&) override;
    virtual void LSL_reg16_RM16(const X86::Instruction&) override;
    virtual void LSL_reg32_RM32(const X86::Instruction&) override;
    virtual void LSS_reg16_mem16(const X86::Instruction&) override;
    virtual void LSS_reg32_mem32(const X86::Instruction&) override;
    virtual void LTR_RM16(const X86::Instruction&) override;
    virtual void MOVSB(const X86::Instruction&) override;
    virtual void MOVSD(const X86::Instruction&) override;
    virtual void MOVSW(const X86::Instruction&) override;
    virtual void MOVSX_reg16_RM8(const X86::Instruction&) override;
    virtual void MOVSX_reg32_RM16(const X86::Instruction&) override;
    virtual void MOVSX_reg32_RM8(const X86::Instruction&) override;
    virtual void MOVZX_reg16_RM8(const X86::Instruction&) override;
    virtual void MOVZX_reg32_RM16(const X86::Instruction&) override;
    virtual void MOVZX_reg32_RM8(const X86::Instruction&) override;
    virtual void MOV_AL_moff8(const X86::Instruction&) override;
    virtual void MOV_AX_moff16(const X86::Instruction&) override;
    virtual void MOV_CR_reg32(const X86::Instruction&) override;
    virtual void MOV_DR_reg32(const X86::Instruction&) override;
    virtual void MOV_EAX_moff32(const X86::Instruction&) override;
    virtual void MOV_RM16_imm16(const X86::Instruction&) override;
    virtual void MOV_RM16_reg16(const X86::Instruction&) override;
    virtual void MOV_RM16_seg(const X86::Instruction&) override;
    virtual void MOV_RM32_imm32(const X86::Instruction&) override;
    virtual void MOV_RM32_reg32(const X86::Instruction&) override;
    virtual void MOV_RM8_imm8(const X86::Instruction&) override;
    virtual void MOV_RM8_reg8(const X86::Instruction&) override;
    virtual void MOV_moff16_AX(const X86::Instruction&) override;
    virtual void MOV_moff32_EAX(const X86::Instruction&) override;
    virtual void MOV_moff8_AL(const X86::Instruction&) override;
    virtual void MOV_reg16_RM16(const X86::Instruction&) override;
    virtual void MOV_reg16_imm16(const X86::Instruction&) override;
    virtual void MOV_reg32_CR(const X86::Instruction&) override;
    virtual void MOV_reg32_DR(const X86::Instruction&) override;
    virtual void MOV_reg32_RM32(const X86::Instruction&) override;
    virtual void MOV_reg32_imm32(const X86::Instruction&) override;
    virtual void MOV_reg8_RM8(const X86::Instruction&) override;
    virtual void MOV_reg8_imm8(const X86::Instruction&) override;
    virtual void MOV_seg_RM16(const X86::Instruction&) override;
    virtual void MOV_seg_RM32(const X86::Instruction&) override;
    virtual void MUL_RM16(const X86::Instruction&) override;
    virtual void MUL_RM32(const X86::Instruction&) override;
    virtual void MUL_RM8(const X86::Instruction&) override;
    virtual void NEG_RM16(const X86::Instruction&) override;
    virtual void NEG_RM32(const X86::Instruction&) override;
    virtual void NEG_RM8(const X86::Instruction&) override;
    virtual void NOP(const X86::Instruction&) override;
    virtual void NOT_RM16(const X86::Instruction&) override;
    virtual void NOT_RM32(const X86::Instruction&) override;
    virtual void NOT_RM8(const X86::Instruction&) override;
    virtual void OR_AL_imm8(const X86::Instruction&) override;
    virtual void OR_AX_imm16(const X86::Instruction&) override;
    virtual void OR_EAX_imm32(const X86::Instruction&) override;
    virtual void OR_RM16_imm16(const X86::Instruction&) override;
    virtual void OR_RM16_imm8(const X86::Instruction&) override;
    virtual void OR_RM16_reg16(const X86::Instruction&) override;
    virtual void OR_RM32_imm32(const X86::Instruction&) override;
    virtual void OR_RM32_imm8(const X86::Instruction&) override;
    virtual void OR_RM32_reg32(const X86::Instruction&) override;
    virtual void OR_RM8_imm8(const X86::Instruction&) override;
    virtual void OR_RM8_reg8(const X86::Instruction&) override;
    virtual void OR_reg16_RM16(const X86::Instruction&) override;
    virtual void OR_reg32_RM32(const X86::Instruction&) override;
    virtual void OR_reg8_RM8(const X86::Instruction&) override;
    virtual void OUTSB(const X86::Instruction&) override;
    virtual void OUTSD(const X86::Instruction&) override;
    virtual void OUTSW(const X86::Instruction&) override;
    virtual void OUT_DX_AL(const X86::Instruction&) override;
    virtual void OUT_DX_AX(const X86::Instruction&) override;
    virtual void OUT_DX_EAX(const X86::Instruction&) override;
    virtual void OUT_imm8_AL(const X86::Instruction&) override;
    virtual void OUT_imm8_AX(const X86::Instruction&) override;
    virtual void OUT_imm8_EAX(const X86::Instruction&) override;
    virtual void PADDB_mm1_mm2m64(const X86::Instruction&) override;
    virtual void PADDW_mm1_mm2m64(const X86::Instruction&) override;
    virtual void PADDD_mm1_mm2m64(const X86::Instruction&) override;
    virtual void POPA(const X86::Instruction&) override;
    virtual void POPAD(const X86::Instruction&) override;
    virtual void POPF(const X86::Instruction&) override;
    virtual void POPFD(const X86::Instruction&) override;
    virtual void POP_DS(const X86::Instruction&) override;
    virtual void POP_ES(const X86::Instruction&) override;
    virtual void POP_FS(const X86::Instruction&) override;
    virtual void POP_GS(const X86::Instruction&) override;
    virtual void POP_RM16(const X86::Instruction&) override;
    virtual void POP_RM32(const X86::Instruction&) override;
    virtual void POP_SS(const X86::Instruction&) override;
    virtual void POP_reg16(const X86::Instruction&) override;
    virtual void POP_reg32(const X86::Instruction&) override;
    virtual void PUSHA(const X86::Instruction&) override;
    virtual void PUSHAD(const X86::Instruction&) override;
    virtual void PUSHF(const X86::Instruction&) override;
    virtual void PUSHFD(const X86::Instruction&) override;
    virtual void PUSH_CS(const X86::Instruction&) override;
    virtual void PUSH_DS(const X86::Instruction&) override;
    virtual void PUSH_ES(const X86::Instruction&) override;
    virtual void PUSH_FS(const X86::Instruction&) override;
    virtual void PUSH_GS(const X86::Instruction&) override;
    virtual void PUSH_RM16(const X86::Instruction&) override;
    virtual void PUSH_RM32(const X86::Instruction&) override;
    virtual void PUSH_SP_8086_80186(const X86::Instruction&) override;
    virtual void PUSH_SS(const X86::Instruction&) override;
    virtual void PUSH_imm16(const X86::Instruction&) override;
    virtual void PUSH_imm32(const X86::Instruction&) override;
    virtual void PUSH_imm8(const X86::Instruction&) override;
    virtual void PUSH_reg16(const X86::Instruction&) override;
    virtual void PUSH_reg32(const X86::Instruction&) override;
    virtual void RCL_RM16_1(const X86::Instruction&) override;
    virtual void RCL_RM16_CL(const X86::Instruction&) override;
    virtual void RCL_RM16_imm8(const X86::Instruction&) override;
    virtual void RCL_RM32_1(const X86::Instruction&) override;
    virtual void RCL_RM32_CL(const X86::Instruction&) override;
    virtual void RCL_RM32_imm8(const X86::Instruction&) override;
    virtual void RCL_RM8_1(const X86::Instruction&) override;
    virtual void RCL_RM8_CL(const X86::Instruction&) override;
    virtual void RCL_RM8_imm8(const X86::Instruction&) override;
    virtual void RCR_RM16_1(const X86::Instruction&) override;
    virtual void RCR_RM16_CL(const X86::Instruction&) override;
    virtual void RCR_RM16_imm8(const X86::Instruction&) override;
    virtual void RCR_RM32_1(const X86::Instruction&) override;
    virtual void RCR_RM32_CL(const X86::Instruction&) override;
    virtual void RCR_RM32_imm8(const X86::Instruction&) override;
    virtual void RCR_RM8_1(const X86::Instruction&) override;
    virtual void RCR_RM8_CL(const X86::Instruction&) override;
    virtual void RCR_RM8_imm8(const X86::Instruction&) override;
    virtual void RDTSC(const X86::Instruction&) override;
    virtual void RET(const X86::Instruction&) override;
    virtual void RETF(const X86::Instruction&) override;
    virtual void RETF_imm16(const X86::Instruction&) override;
    virtual void RET_imm16(const X86::Instruction&) override;
    virtual void ROL_RM16_1(const X86::Instruction&) override;
    virtual void ROL_RM16_CL(const X86::Instruction&) override;
    virtual void ROL_RM16_imm8(const X86::Instruction&) override;
    virtual void ROL_RM32_1(const X86::Instruction&) override;
    virtual void ROL_RM32_CL(const X86::Instruction&) override;
    virtual void ROL_RM32_imm8(const X86::Instruction&) override;
    virtual void ROL_RM8_1(const X86::Instruction&) override;
    virtual void ROL_RM8_CL(const X86::Instruction&) override;
    virtual void ROL_RM8_imm8(const X86::Instruction&) override;
    virtual void ROR_RM16_1(const X86::Instruction&) override;
    virtual void ROR_RM16_CL(const X86::Instruction&) override;
    virtual void ROR_RM16_imm8(const X86::Instruction&) override;
    virtual void ROR_RM32_1(const X86::Instruction&) override;
    virtual void ROR_RM32_CL(const X86::Instruction&) override;
    virtual void ROR_RM32_imm8(const X86::Instruction&) override;
    virtual void ROR_RM8_1(const X86::Instruction&) override;
    virtual void ROR_RM8_CL(const X86::Instruction&) override;
    virtual void ROR_RM8_imm8(const X86::Instruction&) override;
    virtual void SAHF(const X86::Instruction&) override;
    virtual void SALC(const X86::Instruction&) override;
    virtual void SAR_RM16_1(const X86::Instruction&) override;
    virtual void SAR_RM16_CL(const X86::Instruction&) override;
    virtual void SAR_RM16_imm8(const X86::Instruction&) override;
    virtual void SAR_RM32_1(const X86::Instruction&) override;
    virtual void SAR_RM32_CL(const X86::Instruction&) override;
    virtual void SAR_RM32_imm8(const X86::Instruction&) override;
    virtual void SAR_RM8_1(const X86::Instruction&) override;
    virtual void SAR_RM8_CL(const X86::Instruction&) override;
    virtual void SAR_RM8_imm8(const X86::Instruction&) override;
    virtual void SBB_AL_imm8(const X86::Instruction&) override;
    virtual void SBB_AX_imm16(const X86::Instruction&) override;
    virtual void SBB_EAX_imm32(const X86::Instruction&) override;
    virtual void SBB_RM16_imm16(const X86::Instruction&) override;
    virtual void SBB_RM16_imm8(const X86::Instruction&) override;
    virtual void SBB_RM16_reg16(const X86::Instruction&) override;
    virtual void SBB_RM32_imm32(const X86::Instruction&) override;
    virtual void SBB_RM32_imm8(const X86::Instruction&) override;
    virtual void SBB_RM32_reg32(const X86::Instruction&) override;
    virtual void SBB_RM8_imm8(const X86::Instruction&) override;
    virtual void SBB_RM8_reg8(const X86::Instruction&) override;
    virtual void SBB_reg16_RM16(const X86::Instruction&) override;
    virtual void SBB_reg32_RM32(const X86::Instruction&) override;
    virtual void SBB_reg8_RM8(const X86::Instruction&) override;
    virtual void SCASB(const X86::Instruction&) override;
    virtual void SCASD(const X86::Instruction&) override;
    virtual void SCASW(const X86::Instruction&) override;
    virtual void SETcc_RM8(const X86::Instruction&) override;
    virtual void SGDT(const X86::Instruction&) override;
    virtual void SHLD_RM16_reg16_CL(const X86::Instruction&) override;
    virtual void SHLD_RM16_reg16_imm8(const X86::Instruction&) override;
    virtual void SHLD_RM32_reg32_CL(const X86::Instruction&) override;
    virtual void SHLD_RM32_reg32_imm8(const X86::Instruction&) override;
    virtual void SHL_RM16_1(const X86::Instruction&) override;
    virtual void SHL_RM16_CL(const X86::Instruction&) override;
    virtual void SHL_RM16_imm8(const X86::Instruction&) override;
    virtual void SHL_RM32_1(const X86::Instruction&) override;
    virtual void SHL_RM32_CL(const X86::Instruction&) override;
    virtual void SHL_RM32_imm8(const X86::Instruction&) override;
    virtual void SHL_RM8_1(const X86::Instruction&) override;
    virtual void SHL_RM8_CL(const X86::Instruction&) override;
    virtual void SHL_RM8_imm8(const X86::Instruction&) override;
    virtual void SHRD_RM16_reg16_CL(const X86::Instruction&) override;
    virtual void SHRD_RM16_reg16_imm8(const X86::Instruction&) override;
    virtual void SHRD_RM32_reg32_CL(const X86::Instruction&) override;
    virtual void SHRD_RM32_reg32_imm8(const X86::Instruction&) override;
    virtual void SHR_RM16_1(const X86::Instruction&) override;
    virtual void SHR_RM16_CL(const X86::Instruction&) override;
    virtual void SHR_RM16_imm8(const X86::Instruction&) override;
    virtual void SHR_RM32_1(const X86::Instruction&) override;
    virtual void SHR_RM32_CL(const X86::Instruction&) override;
    virtual void SHR_RM32_imm8(const X86::Instruction&) override;
    virtual void SHR_RM8_1(const X86::Instruction&) override;
    virtual void SHR_RM8_CL(const X86::Instruction&) override;
    virtual void SHR_RM8_imm8(const X86::Instruction&) override;
    virtual void SIDT(const X86::Instruction&) override;
    virtual void SLDT_RM16(const X86::Instruction&) override;
    virtual void SMSW_RM16(const X86::Instruction&) override;
    virtual void STC(const X86::Instruction&) override;
    virtual void STD(const X86::Instruction&) override;
    virtual void STI(const X86::Instruction&) override;
    virtual void STOSB(const X86::Instruction&) override;
    virtual void STOSD(const X86::Instruction&) override;
    virtual void STOSW(const X86::Instruction&) override;
    virtual void STR_RM16(const X86::Instruction&) override;
    virtual void SUB_AL_imm8(const X86::Instruction&) override;
    virtual void SUB_AX_imm16(const X86::Instruction&) override;
    virtual void SUB_EAX_imm32(const X86::Instruction&) override;
    virtual void SUB_RM16_imm16(const X86::Instruction&) override;
    virtual void SUB_RM16_imm8(const X86::Instruction&) override;
    virtual void SUB_RM16_reg16(const X86::Instruction&) override;
    virtual void SUB_RM32_imm32(const X86::Instruction&) override;
    virtual void SUB_RM32_imm8(const X86::Instruction&) override;
    virtual void SUB_RM32_reg32(const X86::Instruction&) override;
    virtual void SUB_RM8_imm8(const X86::Instruction&) override;
    virtual void SUB_RM8_reg8(const X86::Instruction&) override;
    virtual void SUB_reg16_RM16(const X86::Instruction&) override;
    virtual void SUB_reg32_RM32(const X86::Instruction&) override;
    virtual void SUB_reg8_RM8(const X86::Instruction&) override;
    virtual void TEST_AL_imm8(const X86::Instruction&) override;
    virtual void TEST_AX_imm16(const X86::Instruction&) override;
    virtual void TEST_EAX_imm32(const X86::Instruction&) override;
    virtual void TEST_RM16_imm16(const X86::Instruction&) override;
    virtual void TEST_RM16_reg16(const X86::Instruction&) override;
    virtual void TEST_RM32_imm32(const X86::Instruction&) override;
    virtual void TEST_RM32_reg32(const X86::Instruction&) override;
    virtual void TEST_RM8_imm8(const X86::Instruction&) override;
    virtual void TEST_RM8_reg8(const X86::Instruction&) override;
    virtual void UD0(const X86::Instruction&) override;
    virtual void UD1(const X86::Instruction&) override;
    virtual void UD2(const X86::Instruction&) override;
    virtual void VERR_RM16(const X86::Instruction&) override;
    virtual void VERW_RM16(const X86::Instruction&) override;
    virtual void WAIT(const X86::Instruction&) override;
    virtual void WBINVD(const X86::Instruction&) override;
    virtual void XADD_RM16_reg16(const X86::Instruction&) override;
    virtual void XADD_RM32_reg32(const X86::Instruction&) override;
    virtual void XADD_RM8_reg8(const X86::Instruction&) override;
    virtual void XCHG_AX_reg16(const X86::Instruction&) override;
    virtual void XCHG_EAX_reg32(const X86::Instruction&) override;
    virtual void XCHG_reg16_RM16(const X86::Instruction&) override;
    virtual void XCHG_reg32_RM32(const X86::Instruction&) override;
    virtual void XCHG_reg8_RM8(const X86::Instruction&) override;
    virtual void XLAT(const X86::Instruction&) override;
    virtual void XOR_AL_imm8(const X86::Instruction&) override;
    virtual void XOR_AX_imm16(const X86::Instruction&) override;
    virtual void XOR_EAX_imm32(const X86::Instruction&) override;
    virtual void XOR_RM16_imm16(const X86::Instruction&) override;
    virtual void XOR_RM16_imm8(const X86::Instruction&) override;
    virtual void XOR_RM16_reg16(const X86::Instruction&) override;
    virtual void XOR_RM32_imm32(const X86::Instruction&) override;
    virtual void XOR_RM32_imm8(const X86::Instruction&) override;
    virtual void XOR_RM32_reg32(const X86::Instruction&) override;
    virtual void XOR_RM8_imm8(const X86::Instruction&) override;
    virtual void XOR_RM8_reg8(const X86::Instruction&) override;
    virtual void XOR_reg16_RM16(const X86::Instruction&) override;
    virtual void XOR_reg32_RM32(const X86::Instruction&) override;
    virtual void XOR_reg8_RM8(const X86::Instruction&) override;
    virtual void MOVQ_mm1_mm2m64(const X86::Instruction&) override;
    virtual void EMMS(const X86::Instruction&) override;
    virtual void MOVQ_mm1_m64_mm2(const X86::Instruction&) override;
    virtual void wrap_0xC0(const X86::Instruction&) override;
    virtual void wrap_0xC1_16(const X86::Instruction&) override;
    virtual void wrap_0xC1_32(const X86::Instruction&) override;
    virtual void wrap_0xD0(const X86::Instruction&) override;
    virtual void wrap_0xD1_16(const X86::Instruction&) override;
    virtual void wrap_0xD1_32(const X86::Instruction&) override;
    virtual void wrap_0xD2(const X86::Instruction&) override;
    virtual void wrap_0xD3_16(const X86::Instruction&) override;
    virtual void wrap_0xD3_32(const X86::Instruction&) override;

    template<bool update_dest, typename Op>
    void generic_AL_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_AX_imm16(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_EAX_imm32(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM16_imm16(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM16_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM16_unsigned_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM16_reg16(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM32_imm32(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM32_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM32_unsigned_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM32_reg32(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM8_imm8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_RM8_reg8(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_reg16_RM16(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_reg32_RM32(Op, const X86::Instruction&);
    template<bool update_dest, typename Op>
    void generic_reg8_RM8(Op, const X86::Instruction&);

    template<typename Op>
    void generic_RM8_1(Op, const X86::Instruction&);
    template<typename Op>
    void generic_RM8_CL(Op, const X86::Instruction&);
    template<typename Op>
    void generic_RM16_1(Op, const X86::Instruction&);
    template<typename Op>
    void generic_RM16_CL(Op, const X86::Instruction&);
    template<typename Op>
    void generic_RM32_1(Op, const X86::Instruction&);
    template<typename Op>
    void generic_RM32_CL(Op, const X86::Instruction&);

    void update_code_cache();

    BIOSEmulator& m_emulator;
    const BIOSEmulatorMemory* m_cached_code { nullptr };
    u8* m_cached_code_base_ptr { nullptr };
    PartAddressableRegister m_gpr[8];
    u16 m_segment[8] { 0 };
    u32 m_eflags { 0 };
    FlatPtr m_eip { 0 };
};

class BIOSEmulator final {
    friend class SoftCPU;
public:
    explicit BIOSEmulator(BIOSEmulatorMemory& bios_ivt, BIOSEmulatorMemory& bios_rom)
        : m_cpu(*this)
    {
        add_memory(bios_ivt);
        add_memory(bios_rom);
    }

    void exec(BIOSEmulatorMemory&, RegisterState&);

private:
    void add_memory(BIOSEmulatorMemory& memory)
    {
        m_memory.insert_before_matching(&memory, [&](BIOSEmulatorMemory* mem) {
                return memory.begin > mem->begin;
            });
    }
    void remove_memory(BIOSEmulatorMemory& memory)
    {
        [[maybe_unused]] bool removed = m_memory.remove_first_matching([&](BIOSEmulatorMemory* mem) {
                return mem == &memory;
            });
        VERIFY(removed);
    }
    template<typename T>
    BIOSEmulatorMemory* find_memory(FlatPtr address)
    {
        for (auto& mem : m_memory) {
            if (address < mem->begin)
                continue;
            if (address + sizeof(T) > mem->end)
                break;
            return mem;
        }
        return nullptr;
    }
    template<typename T>
    void write_memory(FlatPtr address, T value)
    {
        if (auto* memory = find_memory<T>(address); memory && memory->type == BIOSEmulatorMemory::Type::DataReadWrite) {
            memory->write(address - memory->begin, value);
            return;
        }
        // TODO: Handle a write that spans two or more memory locations
        VERIFY_NOT_REACHED();
    }
    template<typename T>
    T read_memory(FlatPtr address)
    {
        if (auto* memory = find_memory<T>(address))
            return memory->template read<T>(address - memory->begin);
        // TODO: Handle a read that spans two or more memory locations
        VERIFY_NOT_REACHED();
    }

    SoftCPU m_cpu;
    Vector<BIOSEmulatorMemory*> m_memory;
};


template<typename... Ts>
void reportln(const StringView& format, Ts... args)
{
//    if (g_report_to_debug)
        AK::vdbgln(format, AK::VariadicFormatParams { args... });
//    else
//        warnln(format, args...);
}

ALWAYS_INLINE u8 SoftCPU::read8()
{
    if (!m_cached_code || !m_cached_code->contains(m_eip))
        update_code_cache();

    u8 value = m_cached_code_base_ptr[m_eip - m_cached_code->begin];
    m_eip += 1;
    return value;
}

ALWAYS_INLINE u16 SoftCPU::read16()
{
    if (!m_cached_code || !m_cached_code->contains(m_eip))
        update_code_cache();

    u16 value = *reinterpret_cast<const u16*>(&m_cached_code_base_ptr[m_eip - m_cached_code->begin]);
    m_eip += 2;
    return value;
}

ALWAYS_INLINE u32 SoftCPU::read32()
{
    if (!m_cached_code || !m_cached_code->contains(m_eip))
        update_code_cache();

    u32 value = *reinterpret_cast<const u32*>(&m_cached_code_base_ptr[m_eip - m_cached_code->begin]);
    m_eip += 4;
    return value;
}

ALWAYS_INLINE u64 SoftCPU::read64()
{
    if (!m_cached_code || !m_cached_code->contains(m_eip))
        update_code_cache();

    auto value = *reinterpret_cast<const u64*>(&m_cached_code_base_ptr[m_eip - m_cached_code->begin]);
    m_eip += 8;
    return value;
}

#define TODO_INSN()                                                                   \
    do {                                                                              \
        dbgln("BIOSEmulator: Unimplemented instruction: {}\n", __FUNCTION__); \
        VERIFY_NOT_REACHED();                                                                     \
    } while (0)

#define DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(mnemonic, op)                                                                            \
    void SoftCPU::mnemonic##_RM8_1(const X86::Instruction& insn) { generic_RM8_1(op<u8>, insn); }                         \
    void SoftCPU::mnemonic##_RM8_CL(const X86::Instruction& insn) { generic_RM8_CL(op<u8>, insn); }                       \
    void SoftCPU::mnemonic##_RM8_imm8(const X86::Instruction& insn) { generic_RM8_imm8<true>(op<u8>, insn); }      \
    void SoftCPU::mnemonic##_RM16_1(const X86::Instruction& insn) { generic_RM16_1(op<u16>, insn); }                      \
    void SoftCPU::mnemonic##_RM16_CL(const X86::Instruction& insn) { generic_RM16_CL(op<u16>, insn); }                    \
    void SoftCPU::mnemonic##_RM16_imm8(const X86::Instruction& insn) { generic_RM16_unsigned_imm8<true>(op<u16>, insn); } \
    void SoftCPU::mnemonic##_RM32_1(const X86::Instruction& insn) { generic_RM32_1(op<u32>, insn); }                      \
    void SoftCPU::mnemonic##_RM32_CL(const X86::Instruction& insn) { generic_RM32_CL(op<u32>, insn); }                    \
    void SoftCPU::mnemonic##_RM32_imm8(const X86::Instruction& insn) { generic_RM32_unsigned_imm8<true>(op<u32>, insn); }


template<class Dest, class Source>
static inline Dest bit_cast(Source source)
{
    static_assert(sizeof(Dest) == sizeof(Source));
    Dest dest;
    memcpy(&dest, &source, sizeof(dest));
    return dest;
}

template<typename T, typename U>
constexpr T sign_extended_to(U value)
{
    if (!(value & X86::TypeTrivia<U>::sign_bit))
        return value;
    return (X86::TypeTrivia<T>::mask & ~X86::TypeTrivia<U>::mask) | value;
}

template<typename T, typename U>
constexpr T sign_extended_to(SoftCPU::Value<U> value)
{
    if (!(value & X86::TypeTrivia<U>::sign_bit))
        return value;
    return (X86::TypeTrivia<T>::mask & ~X86::TypeTrivia<U>::mask) | value;
}

void SoftCPU::dump() const
{
    dbgln(" eax={:08x}  ebx={:08x}  ecx={:08x}  edx={:08x}  ebp={:08x}  esp={:08x}  esi={:08x}  edi={:08x} o={:d} s={:d} z={:d} a={:d} p={:d} c={:d}",
          eax(), ebx(), ecx(), edx(), ebp(), esp(), esi(), edi(), of(), sf(), zf(), af(), pf(), cf());
}

FlatPtr SoftCPU::translate_address(X86::LogicalAddress address)
{
    // TODO: Check CR0.PE
    return (address.selector() << 4) | (address.offset() & 0xffff);
}

u8 SoftCPU::read_memory8(X86::LogicalAddress address)
{
    auto value = m_emulator.read_memory<u8>(translate_address(address));
    dbgln_if(MEMORY_DEBUG, "\033[36;1mread_memory8: @{:04x}:{:08x} -> {:02x}\033[0m", address.selector(), address.offset(), value);
    return value;
}

u16 SoftCPU::read_memory16(X86::LogicalAddress address)
{
    auto value = m_emulator.read_memory<u16>(translate_address(address));
    dbgln_if(MEMORY_DEBUG, "\033[36;1mread_memory16: @{:04x}:{:08x} -> {:04x}\033[0m", address.selector(), address.offset(), value);
    return value;
}

u32 SoftCPU::read_memory32(X86::LogicalAddress address)
{
    auto value = m_emulator.read_memory<u32>(translate_address(address));
    dbgln_if(MEMORY_DEBUG, "\033[36;1mread_memory32: @{:04x}:{:08x} -> {:08x}\033[0m", address.selector(), address.offset(), value);
    return value;
}

u64 SoftCPU::read_memory64(X86::LogicalAddress address)
{
    auto value = m_emulator.read_memory<u64>(translate_address(address));
    dbgln_if(MEMORY_DEBUG, "\033[36;1mread_memory64: @{:04x}:{:08x} -> {:016x}\033[0m", address.selector(), address.offset(), value);
    return value;
}

void SoftCPU::write_memory8(X86::LogicalAddress address, u8 value)
{
    dbgln_if(MEMORY_DEBUG, "\033[36;1mwrite_memory8: @{:04x}:{:08x} <- {:02x}\033[0m", address.selector(), address.offset(), value);
    m_emulator.write_memory<u8>(translate_address(address), value);
}

void SoftCPU::write_memory16(X86::LogicalAddress address, u16 value)
{
    dbgln_if(MEMORY_DEBUG, "\033[36;1mwrite_memory16: @{:04x}:{:08x} <- {:04x}\033[0m", address.selector(), address.offset(), value);
    m_emulator.write_memory<u16>(translate_address(address), value);
}

void SoftCPU::write_memory32(X86::LogicalAddress address, u32 value)
{
    dbgln_if(MEMORY_DEBUG, "\033[36;1mwrite_memory32: @{:04x}:{:08x} <- {:08x}\033[0m", address.selector(), address.offset(), value);
    m_emulator.write_memory<u32>(translate_address(address), value);
}

void SoftCPU::write_memory64(X86::LogicalAddress address, u64 value)
{
    dbgln_if(MEMORY_DEBUG, "\033[36;1mwrite_memory64: @{:04x}:{:08x} <- {:016x}\033[0m", address.selector(), address.offset(), value);
    m_emulator.write_memory<u64>(translate_address(address), value);
}

void SoftCPU::push32(u32 value)
{
    set_esp(esp() - sizeof(u32));
    write_memory32({ ss(), esp() }, value);
}

u32 SoftCPU::pop32()
{
    auto value = read_memory32({ ss(), esp() });
    set_esp(esp() + sizeof(u32));
    return value;
}

void SoftCPU::push16(u16 value)
{
    set_esp(esp() - sizeof(u16));
    write_memory16({ ss(), esp() }, value);
}

u16 SoftCPU::pop16()
{
    auto value = read_memory16({ ss(), esp() });
    set_esp(esp() + sizeof(u16));
    return value;
}

template<bool check_zf, typename Callback>
void SoftCPU::do_once_or_repeat(const X86::Instruction& insn, Callback callback)
{
    if (!insn.has_rep_prefix())
        return callback();

    while (loop_index(insn.a32())) {
        callback();
        decrement_loop_index(insn.a32());
        if constexpr (check_zf) {
            if (insn.rep_prefix() == X86::Prefix::REPZ && !zf())
                break;
            if (insn.rep_prefix() == X86::Prefix::REPNZ && zf())
                break;
        }
    }
}

template<typename T>
ALWAYS_INLINE static T op_inc(SoftCPU& cpu, T data)
{
    T result;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("incl %%eax\n"
        : "=a"(result)
        : "a"(data));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("incw %%ax\n"
        : "=a"(result)
        : "a"(data));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("incb %%al\n"
        : "=a"(result)
        : "a"(data));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszap(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_dec(SoftCPU& cpu, T data)
{
    T result;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("decl %%eax\n"
        : "=a"(result)
        : "a"(data));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("decw %%ax\n"
        : "=a"(result)
        : "a"(data));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("decb %%al\n"
        : "=a"(result)
        : "a"(data));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszap(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_xor(SoftCPU& cpu, const T& dest, const T& src)
{
    T result;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("xorl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("xor %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("xorb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszpc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_or(SoftCPU& cpu, const T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("orl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("or %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("orb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszpc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_sub(SoftCPU& cpu, const T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("subl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("subw %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("subb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T, bool cf>
ALWAYS_INLINE static T op_sbb_impl(SoftCPU& cpu, const T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (cf)
        asm volatile("stc");
    else
        asm volatile("clc");

    if constexpr (sizeof(T) == 4) {
        asm volatile("sbbl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("sbbw %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("sbbb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_sbb(SoftCPU& cpu, T& dest, const T& src)
{
    if (cpu.cf())
        return op_sbb_impl<T, true>(cpu, dest, src);
    return op_sbb_impl<T, false>(cpu, dest, src);
}

template<typename T>
ALWAYS_INLINE static T op_add(SoftCPU& cpu, T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("addl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("addw %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("addb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T, bool cf>
ALWAYS_INLINE static T op_adc_impl(SoftCPU& cpu, T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (cf)
        asm volatile("stc");
    else
        asm volatile("clc");

    if constexpr (sizeof(T) == 4) {
        asm volatile("adcl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("adcw %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("adcb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_adc(SoftCPU& cpu, T& dest, const T& src)
{
    if (cpu.cf())
        return op_adc_impl<T, true>(cpu, dest, src);
    return op_adc_impl<T, false>(cpu, dest, src);
}

template<typename T>
ALWAYS_INLINE static T op_and(SoftCPU& cpu, const T& dest, const T& src)
{
    T result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("andl %%ecx, %%eax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("andw %%cx, %%ax\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("andb %%cl, %%al\n"
        : "=a"(result)
        : "a"(dest), "c"(src));
    } else {
        VERIFY_NOT_REACHED();
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszpc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static void op_imul(SoftCPU& cpu, const T& dest, const T& src, T& result_high, T& result_low)
{
    bool did_overflow = false;
    if constexpr (sizeof(T) == 4) {
        i64 result = (i64)src * (i64)dest;
        result_low = result & 0xffffffff;
        result_high = result >> 32;
        did_overflow = (result > NumericLimits<T>::max() || result < NumericLimits<T>::min());
    } else if constexpr (sizeof(T) == 2) {
        i32 result = (i32)src * (i32)dest;
        result_low = result & 0xffff;
        result_high = result >> 16;
        did_overflow = (result > NumericLimits<T>::max() || result < NumericLimits<T>::min());
    } else if constexpr (sizeof(T) == 1) {
        i16 result = (i16)src * (i16)dest;
        result_low = result & 0xff;
        result_high = result >> 8;
        did_overflow = (result > NumericLimits<T>::max() || result < NumericLimits<T>::min());
    }

    if (did_overflow) {
        cpu.set_cf(true);
        cpu.set_of(true);
    } else {
        cpu.set_cf(false);
        cpu.set_of(false);
    }
}

template<typename T>
ALWAYS_INLINE static T op_shr(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("shrl %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("shrw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("shrb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_shl(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("shll %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("shlw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("shlb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_shrd(SoftCPU& cpu, T data, T extra_bits, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("shrd %%cl, %%edx, %%eax\n"
        : "=a"(result)
        : "a"(data), "d"(extra_bits), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("shrd %%cl, %%dx, %%ax\n"
        : "=a"(result)
        : "a"(data), "d"(extra_bits), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_shld(SoftCPU& cpu, T data, T extra_bits, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("shld %%cl, %%edx, %%eax\n"
        : "=a"(result)
        : "a"(data), "d"(extra_bits), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("shld %%cl, %%dx, %%ax\n"
        : "=a"(result)
        : "a"(data), "d"(extra_bits), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_AL_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = al().value();
    auto src = insn.imm8();
    auto result = op(*this, dest, src);
    if (update_dest)
        set_al(result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_AX_imm16(Op op, const X86::Instruction& insn)
{
    auto dest = ax().value();
    auto src = insn.imm16();
    auto result = op(*this, dest, src);
    if (update_dest)
        set_ax(result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_EAX_imm32(Op op, const X86::Instruction& insn)
{
    auto dest = eax().value();
    auto src = insn.imm32();
    auto result = op(*this, dest, src);
    if (update_dest)
        set_eax(result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_imm16(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read16(*this, insn);
    auto src = insn.imm16();
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write16(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read16(*this, insn);
    auto src = sign_extended_to<u16>(insn.imm8());
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write16(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_unsigned_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read16(*this, insn);
    auto src = insn.imm8();
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write16(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_reg16(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read16(*this, insn);
    auto src = const_gpr16(insn.reg16());
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write16(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_imm32(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read32(*this, insn);
    auto src = insn.imm32();
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write32(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read32(*this, insn);
    auto src = sign_extended_to<u32>(insn.imm8());
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write32(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_unsigned_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read32(*this, insn);
    auto src = insn.imm8();
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write32(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_reg32(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read32(*this, insn);
    auto src = const_gpr32(insn.reg32());
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write32(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM8_imm8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read8(*this, insn);
    auto src = insn.imm8();
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write8(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM8_reg8(Op op, const X86::Instruction& insn)
{
    auto dest = insn.modrm().read8(*this, insn);
    auto src = const_gpr8(insn.reg8());
    auto result = op(*this, dest, src);
    if (update_dest)
        insn.modrm().write8(*this, insn, result);
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_reg16_RM16(Op op, const X86::Instruction& insn)
{
    auto dest = const_gpr16(insn.reg16());
    auto src = insn.modrm().read16(*this, insn);
    auto result = op(*this, dest, src);
    if (update_dest)
        gpr16(insn.reg16()) = result;
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_reg32_RM32(Op op, const X86::Instruction& insn)
{
    auto dest = const_gpr32(insn.reg32());
    auto src = insn.modrm().read32(*this, insn);
    auto result = op(*this, dest, src);
    if (update_dest)
        gpr32(insn.reg32()) = result;
}

template<bool update_dest, typename Op>
ALWAYS_INLINE void SoftCPU::generic_reg8_RM8(Op op, const X86::Instruction& insn)
{
    auto dest = const_gpr8(insn.reg8());
    auto src = insn.modrm().read8(*this, insn);
    auto result = op(*this, dest, src);
    if (update_dest)
        gpr8(insn.reg8()) = result;
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM8_1(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read8(*this, insn);
    insn.modrm().write8(*this, insn, op(*this, data, 1));
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM8_CL(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read8(*this, insn);
    insn.modrm().write8(*this, insn, op(*this, data, cl()));
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_1(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read16(*this, insn);
    insn.modrm().write16(*this, insn, op(*this, data, 1));
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM16_CL(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read16(*this, insn);
    insn.modrm().write16(*this, insn, op(*this, data, cl()));
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_1(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read32(*this, insn);
    insn.modrm().write32(*this, insn, op(*this, data, 1));
}

template<typename Op>
ALWAYS_INLINE void SoftCPU::generic_RM32_CL(Op op, const X86::Instruction& insn)
{
    auto data = insn.modrm().read32(*this, insn);
    insn.modrm().write32(*this, insn, op(*this, data, cl()));
}

void SoftCPU::AAA(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::AAD(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::AAM(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::AAS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::ARPL(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::BOUND(const X86::Instruction&) { TODO_INSN(); }

template<typename T>
ALWAYS_INLINE static T op_bsf(SoftCPU&, T value)
{
    return __builtin_ctz(value);
}

template<typename T>
ALWAYS_INLINE static T op_bsr(SoftCPU&, T value)
{
    T bit_index = 0;
    if constexpr (sizeof(T) == 4) {
        asm volatile("bsrl %%eax, %%edx"
        : "=d"(bit_index)
        : "a"(value));
    }
    if constexpr (sizeof(T) == 2) {
        asm volatile("bsrw %%ax, %%dx"
        : "=d"(bit_index)
        : "a"(value));
    }
    return bit_index;
}

void SoftCPU::BSF_reg16_RM16(const X86::Instruction& insn)
{
    auto src = insn.modrm().read16(*this, insn);
    set_zf(!src);
    if (src)
        gpr16(insn.reg16()) = op_bsf(*this, src);
}

void SoftCPU::BSF_reg32_RM32(const X86::Instruction& insn)
{
    auto src = insn.modrm().read32(*this, insn);
    set_zf(!src);
    if (src)
        gpr32(insn.reg32()) = op_bsf(*this, src);
}

void SoftCPU::BSR_reg16_RM16(const X86::Instruction& insn)
{
    auto src = insn.modrm().read16(*this, insn);
    set_zf(!src);
    if (src)
        gpr16(insn.reg16()) = op_bsr(*this, src);
}

void SoftCPU::BSR_reg32_RM32(const X86::Instruction& insn)
{
    auto src = insn.modrm().read32(*this, insn);
    set_zf(!src);
    if (src)
        gpr32(insn.reg32()) = op_bsr(*this, src);
}

void SoftCPU::BSWAP_reg32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = __builtin_bswap32(gpr32(insn.reg32()));
}

template<typename T>
ALWAYS_INLINE static T op_bt(T value, T)
{
    return value;
}

template<typename T>
ALWAYS_INLINE static T op_bts(T value, T bit_mask)
{
    return value | bit_mask;
}

template<typename T>
ALWAYS_INLINE static T op_btr(T value, T bit_mask)
{
    return value & ~bit_mask;
}

template<typename T>
ALWAYS_INLINE static T op_btc(T value, T bit_mask)
{
    return value ^ bit_mask;
}

template<bool should_update, typename Op>
ALWAYS_INLINE void BTx_RM16_reg16(SoftCPU& cpu, const X86::Instruction& insn, Op op)
{
    if (insn.modrm().is_register()) {
        unsigned bit_index = cpu.const_gpr16(insn.reg16()) & (X86::TypeTrivia<u16>::bits - 1);
        auto original = insn.modrm().read16(cpu, insn);
        u16 bit_mask = 1 << bit_index;
        u16 result = op(original, bit_mask);
        cpu.set_cf((original & bit_mask) != 0);
        if (should_update)
            insn.modrm().write16(cpu, insn, result);
        return;
    }
    // FIXME: Is this supposed to perform a full 16-bit read/modify/write?
    unsigned bit_offset_in_array = cpu.const_gpr16(insn.reg16()) / 8;
    unsigned bit_offset_in_byte = cpu.const_gpr16(insn.reg16()) & 7;
    auto address = insn.modrm().resolve(cpu, insn);
    address.set_offset(address.offset() + bit_offset_in_array);
    auto dest = cpu.read_memory8(address);
    u8 bit_mask = 1 << bit_offset_in_byte;
    u8 result = op(dest, bit_mask);
    cpu.set_cf((dest & bit_mask) != 0);
    if (should_update)
        cpu.write_memory8(address, result);
}

template<bool should_update, typename Op>
ALWAYS_INLINE void BTx_RM32_reg32(SoftCPU& cpu, const X86::Instruction& insn, Op op)
{
    if (insn.modrm().is_register()) {
        unsigned bit_index = cpu.const_gpr32(insn.reg32()) & (X86::TypeTrivia<u32>::bits - 1);
        auto original = insn.modrm().read32(cpu, insn);
        u32 bit_mask = 1 << bit_index;
        u32 result = op(original, bit_mask);
        cpu.set_cf((original & bit_mask) != 0);
        if (should_update)
            insn.modrm().write32(cpu, insn, result);
        return;
    }
    // FIXME: Is this supposed to perform a full 32-bit read/modify/write?
    unsigned bit_offset_in_array = cpu.const_gpr32(insn.reg32()) / 8;
    unsigned bit_offset_in_byte = cpu.const_gpr32(insn.reg32()) & 7;
    auto address = insn.modrm().resolve(cpu, insn);
    address.set_offset(address.offset() + bit_offset_in_array);
    auto dest = cpu.read_memory8(address);
    u8 bit_mask = 1 << bit_offset_in_byte;
    u8 result = op(dest, bit_mask);
    cpu.set_cf((dest & bit_mask) != 0);
    if (should_update)
        cpu.write_memory8(address, result);
}

template<bool should_update, typename Op>
ALWAYS_INLINE void BTx_RM16_imm8(SoftCPU& cpu, const X86::Instruction& insn, Op op)
{
    unsigned bit_index = insn.imm8() & (X86::TypeTrivia<u16>::mask);

    // FIXME: Support higher bit indices
    VERIFY(bit_index < 16);

    auto original = insn.modrm().read16(cpu, insn);
    u16 bit_mask = 1 << bit_index;
    auto result = op(original, bit_mask);
    cpu.set_cf((original & bit_mask) != 0);
    if (should_update)
        insn.modrm().write16(cpu, insn, result);
}

template<bool should_update, typename Op>
ALWAYS_INLINE void BTx_RM32_imm8(SoftCPU& cpu, const X86::Instruction& insn, Op op)
{
    unsigned bit_index = insn.imm8() & (X86::TypeTrivia<u32>::mask);

    // FIXME: Support higher bit indices
    VERIFY(bit_index < 32);

    auto original = insn.modrm().read32(cpu, insn);
    u32 bit_mask = 1 << bit_index;
    auto result = op(original, bit_mask);
    cpu.set_cf((original & bit_mask) != 0);
    if (should_update)
        insn.modrm().write32(cpu, insn, result);
}

#define DEFINE_GENERIC_BTx_INSN_HANDLERS(mnemonic, op, update_dest)                                                          \
    void SoftCPU::mnemonic##_RM32_reg32(const X86::Instruction& insn) { BTx_RM32_reg32<update_dest>(*this, insn, op<u32>); } \
    void SoftCPU::mnemonic##_RM16_reg16(const X86::Instruction& insn) { BTx_RM16_reg16<update_dest>(*this, insn, op<u16>); } \
    void SoftCPU::mnemonic##_RM32_imm8(const X86::Instruction& insn) { BTx_RM32_imm8<update_dest>(*this, insn, op<u32>); }   \
    void SoftCPU::mnemonic##_RM16_imm8(const X86::Instruction& insn) { BTx_RM16_imm8<update_dest>(*this, insn, op<u16>); }

DEFINE_GENERIC_BTx_INSN_HANDLERS(BTS, op_bts, true);
DEFINE_GENERIC_BTx_INSN_HANDLERS(BTR, op_btr, true);
DEFINE_GENERIC_BTx_INSN_HANDLERS(BTC, op_btc, true);
DEFINE_GENERIC_BTx_INSN_HANDLERS(BT, op_bt, false);

void SoftCPU::CALL_FAR_mem16(const X86::Instruction&)
{
        TODO();
}
void SoftCPU::CALL_FAR_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::CALL_RM16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::CALL_RM32(const X86::Instruction& insn)
{
    push32(eip());
    auto address = insn.modrm().read32(*this, insn);
    set_eip(address);
}

void SoftCPU::CALL_imm16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::CALL_imm16_imm16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::CALL_imm16_imm32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::CALL_imm32(const X86::Instruction& insn)
{
    push32(eip());
    set_eip(eip() + (i32)insn.imm32());
}

void SoftCPU::CBW(const X86::Instruction&)
{
    set_ah((al() & 0x80) ? 0xff : 0x00);
}

void SoftCPU::CDQ(const X86::Instruction&)
{
    if (eax() & 0x80000000)
        set_edx(0xffffffff);
    else
        set_edx(0);
}

void SoftCPU::CLC(const X86::Instruction&)
{
    set_cf(false);
}

void SoftCPU::CLD(const X86::Instruction&)
{
    set_df(false);
}

void SoftCPU::CLI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::CLTS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::CMC(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::CMOVcc_reg16_RM16(const X86::Instruction& insn)
{
    if (evaluate_condition(insn.cc()))
        gpr16(insn.reg16()) = insn.modrm().read16(*this, insn);
}

void SoftCPU::CMOVcc_reg32_RM32(const X86::Instruction& insn)
{
    if (evaluate_condition(insn.cc()))
        gpr32(insn.reg32()) = insn.modrm().read32(*this, insn);
}

template<typename T>
ALWAYS_INLINE static void do_cmps(SoftCPU& cpu, const X86::Instruction& insn)
{
    auto src_segment = cpu.segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS));
    cpu.do_once_or_repeat<true>(insn, [&] {
        auto src = cpu.read_memory<T>({ src_segment, cpu.source_index(insn.a32()) });
        auto dest = cpu.read_memory<T>({ cpu.es(), cpu.destination_index(insn.a32()) });
        op_sub(cpu, dest, src);
        cpu.step_source_index(insn.a32(), sizeof(T));
        cpu.step_destination_index(insn.a32(), sizeof(T));
    });
}

void SoftCPU::CMPSB(const X86::Instruction& insn)
{
    do_cmps<u8>(*this, insn);
}

void SoftCPU::CMPSD(const X86::Instruction& insn)
{
    do_cmps<u32>(*this, insn);
}

void SoftCPU::CMPSW(const X86::Instruction& insn)
{
    do_cmps<u16>(*this, insn);
}

void SoftCPU::CMPXCHG_RM16_reg16(const X86::Instruction& insn)
{
    auto current = insn.modrm().read16(*this, insn);
    if (current == ax()) {
        set_zf(true);
        insn.modrm().write16(*this, insn, const_gpr16(insn.reg16()));
    } else {
        set_zf(false);
        set_ax(current);
    }
}

void SoftCPU::CMPXCHG_RM32_reg32(const X86::Instruction& insn)
{
    auto current = insn.modrm().read32(*this, insn);
    if (current == eax()) {
        set_zf(true);
        insn.modrm().write32(*this, insn, const_gpr32(insn.reg32()));
    } else {
        set_zf(false);
        set_eax(current);
    }
}

void SoftCPU::CMPXCHG_RM8_reg8(const X86::Instruction& insn)
{
    auto current = insn.modrm().read8(*this, insn);
    if (current == al()) {
        set_zf(true);
        insn.modrm().write8(*this, insn, const_gpr8(insn.reg8()));
    } else {
        set_zf(false);
        set_al(current);
    }
}

void SoftCPU::CPUID(const X86::Instruction&)
{
    if (eax() == 0) {
        set_eax(1);
        set_ebx(0x6c6c6548);
        set_edx(0x6972466f);
        set_ecx(0x73646e65);
        return;
    }

    if (eax() == 1) {
        u32 stepping = 0;
        u32 model = 1;
        u32 family = 3;
        u32 type = 0;
        set_eax(stepping | (model << 4) | (family << 8) | (type << 12));
        set_ebx(0);
        set_edx((1 << 15)); // Features (CMOV)
        set_ecx(0);
        return;
    }

    dbgln("Unhandled CPUID with eax={:08x}", eax());
}

void SoftCPU::CWD(const X86::Instruction&)
{
    set_dx((ax() & 0x8000) ? 0xffff : 0x0000);
}

void SoftCPU::CWDE(const X86::Instruction&)
{
    set_eax(sign_extended_to<u32>(ax()));
}

void SoftCPU::DAA(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::DAS(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::DEC_RM16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_dec(*this, insn.modrm().read16(*this, insn)));
}

void SoftCPU::DEC_RM32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_dec(*this, insn.modrm().read32(*this, insn)));
}

void SoftCPU::DEC_RM8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, op_dec(*this, insn.modrm().read8(*this, insn)));
}

void SoftCPU::DEC_reg16(const X86::Instruction& insn)
{
    gpr16(insn.reg16()) = op_dec(*this, const_gpr16(insn.reg16()));
}

void SoftCPU::DEC_reg32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = op_dec(*this, const_gpr32(insn.reg32()));
}

void SoftCPU::DIV_RM16(const X86::Instruction& insn)
{
    auto divisor = insn.modrm().read16(*this, insn);
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    u32 dividend = ((u32)dx() << 16) | ax();
    auto quotient = dividend / divisor;
    if (quotient > NumericLimits<u16>::max()) {
        reportln("Divide overflow");
            TODO();
    }

    auto remainder = dividend % divisor;

    set_ax(quotient);
    set_dx(remainder);
}

void SoftCPU::DIV_RM32(const X86::Instruction& insn)
{
    auto divisor = insn.modrm().read32(*this, insn);
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    u64 dividend = ((u64)edx() << 32) | eax();
    auto quotient = dividend / divisor;
    if (quotient > NumericLimits<u32>::max()) {
        reportln("Divide overflow");
            TODO();
    }

    auto remainder = dividend % divisor;

    set_eax(quotient);
    set_edx(remainder);
}

void SoftCPU::DIV_RM8(const X86::Instruction& insn)
{
    auto divisor = insn.modrm().read8(*this, insn);
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    u16 dividend = ax();
    auto quotient = dividend / divisor;
    if (quotient > NumericLimits<u8>::max()) {
        reportln("Divide overflow");
            TODO();
    }

    auto remainder = dividend % divisor;

    set_al(quotient);
    set_ah(remainder);
}

void SoftCPU::ENTER16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::ENTER32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::ESCAPE(const X86::Instruction&)
{
    dbgln("BIOSEmulator: FIXME: x87 floating-point support");
    TODO();
}

void SoftCPU::FADD_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FMUL_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOM_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOMP_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUB_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUBR_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIV_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIVR_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLD_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FXCH(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FST_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNOP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSTP_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDENV(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCHS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FABS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FTST(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FXAM(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDCW(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLD1(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDL2T(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDL2E(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDPI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDLG2(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDLN2(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLDZ(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSTENV(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::F2XM1(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FYL2X(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FPTAN(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FPATAN(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FXTRACT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FPREM1(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDECSTP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FINCSTP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSTCW(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FPREM(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FYL2XP1(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSQRT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSINCOS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FRNDINT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSCALE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSIN(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIADD_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVB(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIMUL_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FICOM_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVBE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FICOMP_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVU(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISUB_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISUBR_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FUCOMPP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIDIV_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIDIVR_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FILD_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVNB(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTTP_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVNE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIST_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVNBE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTP_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCMOVNU(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNENI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNDISI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNCLEX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNINIT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSETPM(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLD_RM80(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FUCOMI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOMI(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSTP_RM80(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FADD_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FMUL_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOM_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOMP_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUB_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUBR_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIV_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIVR_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FLD_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FFREE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTTP_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FST_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSTP_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FRSTOR(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FUCOM(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FUCOMP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSAVE(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSTSW(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIADD_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FADDP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIMUL_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FMULP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FICOM_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FICOMP_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOMPP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISUB_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUBRP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISUBR_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FSUBP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIDIV_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIVRP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIDIVR_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FDIVP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FILD_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FFREEP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTTP_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FIST_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTP_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FBLD_M80(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FNSTSW_AX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FILD_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FUCOMIP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FBSTP_M80(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FCOMIP(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::FISTP_RM64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::HLT(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::IDIV_RM16(const X86::Instruction& insn)
{
    auto divisor_with_shadow = insn.modrm().read16(*this, insn);
    auto divisor = (i16)divisor_with_shadow;
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    i32 dividend = (i32)(((u32)dx() << 16) | (u32)ax());
    i32 result = dividend / divisor;
    if (result > NumericLimits<i16>::max() || result < NumericLimits<i16>::min()) {
        reportln("Divide overflow");
            TODO();
    }

    set_ax(result);
    set_dx(dividend % divisor);
}

void SoftCPU::IDIV_RM32(const X86::Instruction& insn)
{
    auto divisor_with_shadow = insn.modrm().read32(*this, insn);
    auto divisor = (i32)divisor_with_shadow;
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    i64 dividend = (i64)(((u64)edx() << 32) | (u64)eax());
    i64 result = dividend / divisor;
    if (result > NumericLimits<i32>::max() || result < NumericLimits<i32>::min()) {
        reportln("Divide overflow");
            TODO();
    }

    set_eax(result);
    set_edx(dividend % divisor);
}

void SoftCPU::IDIV_RM8(const X86::Instruction& insn)
{
    auto divisor_with_shadow = insn.modrm().read8(*this, insn);
    auto divisor = (i8)divisor_with_shadow;
    if (divisor == 0) {
        reportln("Divide by zero");
            TODO();
    }
    i16 dividend = ax();
    i16 result = dividend / divisor;
    if (result > NumericLimits<i8>::max() || result < NumericLimits<i8>::min()) {
        reportln("Divide overflow");
            TODO();
    }

    set_al(result);
    set_ah(dividend % divisor);
}

void SoftCPU::IMUL_RM16(const X86::Instruction& insn)
{
    i16 result_high;
    i16 result_low;
    auto src = insn.modrm().read16(*this, insn);
    op_imul<i16>(*this, src, ax(), result_high, result_low);
    gpr16(X86::RegisterDX) = result_high;
    gpr16(X86::RegisterAX) = result_low;
}

void SoftCPU::IMUL_RM32(const X86::Instruction& insn)
{
    i32 result_high;
    i32 result_low;
    auto src = insn.modrm().read32(*this, insn);
    op_imul<i32>(*this, src, eax(), result_high, result_low);
    gpr32(X86::RegisterEDX) = result_high;
    gpr32(X86::RegisterEAX) = result_low;
}

void SoftCPU::IMUL_RM8(const X86::Instruction& insn)
{
    i8 result_high;
    i8 result_low;
    auto src = insn.modrm().read8(*this, insn);
    op_imul<i8>(*this, src, al(), result_high, result_low);
    gpr8(X86::RegisterAH) = result_high;
    gpr8(X86::RegisterAL) = result_low;
}

void SoftCPU::IMUL_reg16_RM16(const X86::Instruction& insn)
{
    i16 result_high;
    i16 result_low;
    auto src = insn.modrm().read16(*this, insn);
    op_imul<i16>(*this, gpr16(insn.reg16()), src, result_high, result_low);
    gpr16(insn.reg16()) = result_low;
}

void SoftCPU::IMUL_reg16_RM16_imm16(const X86::Instruction& insn)
{
    i16 result_high;
    i16 result_low;
    auto src = insn.modrm().read16(*this, insn);
    op_imul<i16>(*this, src, insn.imm16(), result_high, result_low);
    gpr16(insn.reg16()) = result_low;
}

void SoftCPU::IMUL_reg16_RM16_imm8(const X86::Instruction& insn)
{
    i16 result_high;
    i16 result_low;
    auto src = insn.modrm().read16(*this, insn);
    op_imul<i16>(*this, src, sign_extended_to<i16>(insn.imm8()), result_high, result_low);
    gpr16(insn.reg16()) = result_low;
}

void SoftCPU::IMUL_reg32_RM32(const X86::Instruction& insn)
{
    i32 result_high;
    i32 result_low;
    auto src = insn.modrm().read32(*this, insn);
    op_imul<i32>(*this, gpr32(insn.reg32()), src, result_high, result_low);
    gpr32(insn.reg32()) = result_low;
}

void SoftCPU::IMUL_reg32_RM32_imm32(const X86::Instruction& insn)
{
    i32 result_high;
    i32 result_low;
    auto src = insn.modrm().read32(*this, insn);
    op_imul<i32>(*this, src, insn.imm32(), result_high, result_low);
    gpr32(insn.reg32()) = result_low;
}

void SoftCPU::IMUL_reg32_RM32_imm8(const X86::Instruction& insn)
{
    i32 result_high;
    i32 result_low;
    auto src = insn.modrm().read32(*this, insn);
    op_imul<i32>(*this, src, sign_extended_to<i32>(insn.imm8()), result_high, result_low);
    gpr32(insn.reg32()) = result_low;
}

void SoftCPU::INC_RM16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_inc(*this, insn.modrm().read16(*this, insn)));
}

void SoftCPU::INC_RM32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_inc(*this, insn.modrm().read32(*this, insn)));
}

void SoftCPU::INC_RM8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, op_inc(*this, insn.modrm().read8(*this, insn)));
}

void SoftCPU::INC_reg16(const X86::Instruction& insn)
{
    gpr16(insn.reg16()) = op_inc(*this, const_gpr16(insn.reg16()));
}

void SoftCPU::INC_reg32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = op_inc(*this, const_gpr32(insn.reg32()));
}

void SoftCPU::INSB(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::INSD(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::INSW(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::INT3(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::INTO(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::INT_imm8(const X86::Instruction& insn)
{
    VERIFY(insn.imm8() == 0x82);
    set_eax(m_emulator.virt_syscall(eax(), edx(), ecx(), ebx()));
}

void SoftCPU::INVLPG(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_AL_DX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_AL_imm8(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_AX_DX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_AX_imm8(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_EAX_DX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IN_EAX_imm8(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::IRET(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::JCXZ_imm8(const X86::Instruction& insn)
{
    if (insn.a32()) {
        if (ecx() == 0)
            set_eip(eip() + (i8)insn.imm8());
    } else {
        if (cx() == 0)
            set_eip(eip() + (i8)insn.imm8());
    }
}

void SoftCPU::JMP_FAR_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::JMP_FAR_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::JMP_RM16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::JMP_RM32(const X86::Instruction& insn)
{
    set_eip(insn.modrm().read32(*this, insn));
}

void SoftCPU::JMP_imm16(const X86::Instruction& insn)
{
    set_eip(eip() + (i16)insn.imm16());
}

void SoftCPU::JMP_imm16_imm16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::JMP_imm16_imm32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::JMP_imm32(const X86::Instruction& insn)
{
    set_eip(eip() + (i32)insn.imm32());
}

void SoftCPU::JMP_short_imm8(const X86::Instruction& insn)
{
    set_eip(eip() + (i8)insn.imm8());
}

void SoftCPU::Jcc_NEAR_imm(const X86::Instruction& insn)
{
    if (evaluate_condition(insn.cc()))
        set_eip(eip() + (i32)insn.imm32());
}

void SoftCPU::Jcc_imm8(const X86::Instruction& insn)
{
    if (evaluate_condition(insn.cc()))
        set_eip(eip() + (i8)insn.imm8());
}

void SoftCPU::LAHF(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LAR_reg16_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LAR_reg32_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LDS_reg16_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LDS_reg32_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LEAVE16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::LEAVE32(const X86::Instruction&)
{
    auto new_ebp = read_memory32({ ss(), ebp() });
    set_esp( ebp() + 4);
    set_ebp(new_ebp);
}

void SoftCPU::LEA_reg16_mem16(const X86::Instruction& insn)
{
    // FIXME: Respect shadow values
    gpr16(insn.reg16()) = insn.modrm().resolve(*this, insn).offset();
}

void SoftCPU::LEA_reg32_mem32(const X86::Instruction& insn)
{
    // FIXME: Respect shadow values
    gpr32(insn.reg32()) = insn.modrm().resolve(*this, insn).offset();
}

void SoftCPU::LES_reg16_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LES_reg32_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LFS_reg16_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LFS_reg32_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LGDT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LGS_reg16_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LGS_reg32_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LIDT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LLDT_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LMSW_RM16(const X86::Instruction&) { TODO_INSN(); }

template<typename T>
ALWAYS_INLINE static void do_lods(SoftCPU& cpu, const X86::Instruction& insn)
{
    auto src_segment = cpu.segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS));
    cpu.do_once_or_repeat<true>(insn, [&] {
        auto src = cpu.read_memory<T>({ src_segment, cpu.source_index(insn.a32()) });
        cpu.gpr<T>(X86::RegisterAL) = src;
        cpu.step_source_index(insn.a32(), sizeof(T));
    });
}

void SoftCPU::LODSB(const X86::Instruction& insn)
{
    do_lods<u8>(*this, insn);
}

void SoftCPU::LODSD(const X86::Instruction& insn)
{
    do_lods<u32>(*this, insn);
}

void SoftCPU::LODSW(const X86::Instruction& insn)
{
    do_lods<u16>(*this, insn);
}

void SoftCPU::LOOPNZ_imm8(const X86::Instruction& insn)
{
    if (insn.a32()) {
        set_ecx(ecx() - 1);
        if (ecx() != 0 && !zf())
            set_eip(eip() + (i8)insn.imm8());
    } else {
        set_cx((u16)(cx() - 1));
        if (cx() != 0 && !zf())
            set_eip(eip() + (i8)insn.imm8());
    }
}
void SoftCPU::LOOPZ_imm8(const X86::Instruction& insn)
{
    if (insn.a32()) {
        set_ecx(ecx() - 1);
        if (ecx() != 0 && zf())
            set_eip(eip() + (i8)insn.imm8());
    } else {
        set_cx((u16)(cx() - 1));
        if (cx() != 0 && zf())
            set_eip(eip() + (i8)insn.imm8());
    }
}

void SoftCPU::LOOP_imm8(const X86::Instruction& insn)
{
    if (insn.a32()) {
        set_ecx(ecx() - 1);
        if (ecx() != 0)
            set_eip(eip() + (i8)insn.imm8());
    } else {
        set_cx((u16)(cx() - 1));
        if (cx() != 0)
            set_eip(eip() + (i8)insn.imm8());
    }
}

void SoftCPU::LSL_reg16_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LSL_reg32_RM32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LSS_reg16_mem16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LSS_reg32_mem32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::LTR_RM16(const X86::Instruction&) { TODO_INSN(); }

template<typename T>
ALWAYS_INLINE static void do_movs(SoftCPU& cpu, const X86::Instruction& insn)
{
    auto src_segment = cpu.segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS));
    cpu.do_once_or_repeat<false>(insn, [&] {
        auto src = cpu.read_memory<T>({ src_segment, cpu.source_index(insn.a32()) });
        cpu.write_memory<T>({ cpu.es(), cpu.destination_index(insn.a32()) }, src);
        cpu.step_source_index(insn.a32(), sizeof(T));
        cpu.step_destination_index(insn.a32(), sizeof(T));
    });
}

void SoftCPU::MOVSB(const X86::Instruction& insn)
{
    do_movs<u8>(*this, insn);
}

void SoftCPU::MOVSD(const X86::Instruction& insn)
{
    do_movs<u32>(*this, insn);
}

void SoftCPU::MOVSW(const X86::Instruction& insn)
{
    do_movs<u16>(*this, insn);
}

void SoftCPU::MOVSX_reg16_RM8(const X86::Instruction& insn)
{
    auto src = insn.modrm().read8(*this, insn);
    gpr16(insn.reg16()) = sign_extended_to<u16>(src);
}

void SoftCPU::MOVSX_reg32_RM16(const X86::Instruction& insn)
{
    auto src = insn.modrm().read16(*this, insn);
    gpr32(insn.reg32()) = sign_extended_to<u32>(src);
}

void SoftCPU::MOVSX_reg32_RM8(const X86::Instruction& insn)
{
    auto src = insn.modrm().read8(*this, insn);
    gpr32(insn.reg32()) = sign_extended_to<u32>(src);
}

void SoftCPU::MOVZX_reg16_RM8(const X86::Instruction& insn)
{
    auto src = insn.modrm().read8(*this, insn);
    gpr16(insn.reg16()) = src;
}

void SoftCPU::MOVZX_reg32_RM16(const X86::Instruction& insn)
{
    auto src = insn.modrm().read16(*this, insn);
    gpr32(insn.reg32()) = src;
}

void SoftCPU::MOVZX_reg32_RM8(const X86::Instruction& insn)
{
    auto src = insn.modrm().read8(*this, insn);
    gpr32(insn.reg32()) = src;
}

void SoftCPU::MOV_AL_moff8(const X86::Instruction& insn)
{
    set_al(read_memory8({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }));
}

void SoftCPU::MOV_AX_moff16(const X86::Instruction& insn)
{
    set_ax(read_memory16({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }));
}

void SoftCPU::MOV_CR_reg32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::MOV_DR_reg32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::MOV_EAX_moff32(const X86::Instruction& insn)
{
    set_eax(read_memory32({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }));
}

void SoftCPU::MOV_RM16_imm16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, insn.imm16());
}

void SoftCPU::MOV_RM16_reg16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, const_gpr16(insn.reg16()));
}

void SoftCPU::MOV_RM16_seg(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::MOV_RM32_imm32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, insn.imm32());
}

void SoftCPU::MOV_RM32_reg32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, const_gpr32(insn.reg32()));
}

void SoftCPU::MOV_RM8_imm8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, insn.imm8());
}

void SoftCPU::MOV_RM8_reg8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, const_gpr8(insn.reg8()));
}

void SoftCPU::MOV_moff16_AX(const X86::Instruction& insn)
{
    write_memory16({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }, ax());
}

void SoftCPU::MOV_moff32_EAX(const X86::Instruction& insn)
{
    write_memory32({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }, eax());
}

void SoftCPU::MOV_moff8_AL(const X86::Instruction& insn)
{
    write_memory8({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), insn.imm_address() }, al());
}

void SoftCPU::MOV_reg16_RM16(const X86::Instruction& insn)
{
    gpr16(insn.reg16()) = insn.modrm().read16(*this, insn);
}

void SoftCPU::MOV_reg16_imm16(const X86::Instruction& insn)
{
    gpr16(insn.reg16()) = insn.imm16();
}

void SoftCPU::MOV_reg32_CR(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::MOV_reg32_DR(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::MOV_reg32_RM32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = insn.modrm().read32(*this, insn);
}

void SoftCPU::MOV_reg32_imm32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = insn.imm32();
}

void SoftCPU::MOV_reg8_RM8(const X86::Instruction& insn)
{
    gpr8(insn.reg8()) = insn.modrm().read8(*this, insn);
}

void SoftCPU::MOV_reg8_imm8(const X86::Instruction& insn)
{
    gpr8(insn.reg8()) = insn.imm8();
}

void SoftCPU::MOV_seg_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::MOV_seg_RM32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::MUL_RM16(const X86::Instruction& insn)
{
    auto src = insn.modrm().read16(*this, insn);
    u32 result = (u32)ax() * (u32)src;

    set_ax(result & 0xffff);
    set_dx(result >> 16);

    set_cf(dx() != 0);
    set_of(dx() != 0);
}

void SoftCPU::MUL_RM32(const X86::Instruction& insn)
{
    auto src = insn.modrm().read32(*this, insn);
    u64 result = (u64)eax() * (u64)src;

    set_eax(result);
    set_edx(result >> 32);

    set_cf(edx() != 0);
    set_of(edx() != 0);
}

void SoftCPU::MUL_RM8(const X86::Instruction& insn)
{
    auto src = insn.modrm().read8(*this, insn);
    u16 result = (u16)al() * src;

    set_ax(result);

    set_cf((result & 0xff00) != 0);
    set_of((result & 0xff00) != 0);
}

void SoftCPU::NEG_RM16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_sub<u16>(*this, 0, insn.modrm().read16(*this, insn)));
}

void SoftCPU::NEG_RM32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_sub<u32>(*this, 0, insn.modrm().read32(*this, insn)));
}

void SoftCPU::NEG_RM8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, op_sub<u8>(*this, 0, insn.modrm().read8(*this, insn)));
}

void SoftCPU::NOP(const X86::Instruction&)
{
}

void SoftCPU::NOT_RM16(const X86::Instruction& insn)
{
    auto data = insn.modrm().read16(*this, insn);
    insn.modrm().write16(*this, insn, ~data);
}

void SoftCPU::NOT_RM32(const X86::Instruction& insn)
{
    auto data = insn.modrm().read32(*this, insn);
    insn.modrm().write32(*this, insn, ~data);
}

void SoftCPU::NOT_RM8(const X86::Instruction& insn)
{
    auto data = insn.modrm().read8(*this, insn);
    insn.modrm().write8(*this, insn, ~data);
}

void SoftCPU::OUTSB(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUTSD(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUTSW(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_DX_AL(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_DX_AX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_DX_EAX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_imm8_AL(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_imm8_AX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::OUT_imm8_EAX(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PADDB_mm1_mm2m64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PADDW_mm1_mm2m64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PADDD_mm1_mm2m64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POPA(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POPAD(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POPF(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::POPFD(const X86::Instruction&)
{
    auto popped_value = pop32();
    m_eflags &= ~0x00fcffff;
    m_eflags |= popped_value & 0x00fcffff;
}

void SoftCPU::POP_DS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POP_ES(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POP_FS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::POP_GS(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::POP_RM16(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, pop16());
}

void SoftCPU::POP_RM32(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, pop32());
}

void SoftCPU::POP_SS(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::POP_reg16(const X86::Instruction& insn)
{
    gpr16(insn.reg16()) = pop16();
}

void SoftCPU::POP_reg32(const X86::Instruction& insn)
{
    gpr32(insn.reg32()) = pop32();
}

void SoftCPU::PUSHA(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSHAD(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSHF(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::PUSHFD(const X86::Instruction&)
{
    // FIXME: Respect shadow flags when they exist!
    push32(m_eflags & 0x00fcffff);
}

void SoftCPU::PUSH_CS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_DS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_ES(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_FS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_GS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_RM16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::PUSH_RM32(const X86::Instruction& insn)
{
    push32(insn.modrm().read32(*this, insn));
}

void SoftCPU::PUSH_SP_8086_80186(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::PUSH_SS(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::PUSH_imm16(const X86::Instruction& insn)
{
    push16(insn.imm16());
}

void SoftCPU::PUSH_imm32(const X86::Instruction& insn)
{
    push32(insn.imm32());
}

void SoftCPU::PUSH_imm8(const X86::Instruction& insn)
{
    VERIFY(!insn.has_operand_size_override_prefix());
    push32(sign_extended_to<i32>(insn.imm8()));
}

void SoftCPU::PUSH_reg16(const X86::Instruction& insn)
{
    push16(gpr16(insn.reg16()));
}

void SoftCPU::PUSH_reg32(const X86::Instruction& insn)
{
    push32(gpr32(insn.reg32()));
}

template<typename T, bool cf>
ALWAYS_INLINE static T op_rcl_impl(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (cf)
        asm volatile("stc");
    else
        asm volatile("clc");

    if constexpr (sizeof(T) == 4) {
        asm volatile("rcll %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("rclw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("rclb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_rcl(SoftCPU& cpu, T data, u8 steps)
{
    if (cpu.cf())
        return op_rcl_impl<T, true>(cpu, data, steps);
    return op_rcl_impl<T, false>(cpu, data, steps);
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(RCL, op_rcl)

template<typename T, bool cf>
ALWAYS_INLINE static T op_rcr_impl(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (cf)
        asm volatile("stc");
    else
        asm volatile("clc");

    if constexpr (sizeof(T) == 4) {
        asm volatile("rcrl %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("rcrw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("rcrb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oc(new_flags);
    return result;
}

template<typename T>
ALWAYS_INLINE static T op_rcr(SoftCPU& cpu, T data, u8 steps)
{
    if (cpu.cf())
        return op_rcr_impl<T, true>(cpu, data, steps);
    return op_rcr_impl<T, false>(cpu, data, steps);
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(RCR, op_rcr)

void SoftCPU::RDTSC(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::RET(const X86::Instruction& insn)
{
    VERIFY(!insn.has_operand_size_override_prefix());
    auto ret_address = pop32();
    set_eip(ret_address);
}

void SoftCPU::RETF(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::RETF_imm16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::RET_imm16(const X86::Instruction& insn)
{
    VERIFY(!insn.has_operand_size_override_prefix());
    auto ret_address = pop32();
    set_eip(ret_address);
    set_esp( esp() + insn.imm16());
}

template<typename T>
ALWAYS_INLINE static T op_rol(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("roll %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("rolw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("rolb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oc(new_flags);
    return result;
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(ROL, op_rol)

template<typename T>
ALWAYS_INLINE static T op_ror(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("rorl %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("rorw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("rorb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oc(new_flags);
    return result;
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(ROR, op_ror)

void SoftCPU::SAHF(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::SALC(const X86::Instruction&)
{
    // FIXME: Respect shadow flags once they exists!
    set_al(cf() ? 0xff : 0x00);
}

template<typename T>
static T op_sar(SoftCPU& cpu, T data, u8 steps)
{
    if (steps == 0)
        return data;

    u32 result = 0;
    u32 new_flags = 0;

    if constexpr (sizeof(T) == 4) {
        asm volatile("sarl %%cl, %%eax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 2) {
        asm volatile("sarw %%cl, %%ax\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    } else if constexpr (sizeof(T) == 1) {
        asm volatile("sarb %%cl, %%al\n"
        : "=a"(result)
        : "a"(data), "c"(steps));
    }

    asm volatile(
    "pushf\n"
    "pop %%ebx"
    : "=b"(new_flags));

    cpu.set_flags_oszapc(new_flags);
    return result;
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(SAR, op_sar)

template<typename T>
ALWAYS_INLINE static void do_scas(SoftCPU& cpu, const X86::Instruction& insn)
{
    cpu.do_once_or_repeat<true>(insn, [&] {
        auto src = cpu.const_gpr<T>(X86::RegisterAL);
        auto dest = cpu.read_memory<T>({ cpu.es(), cpu.destination_index(insn.a32()) });
        op_sub(cpu, dest, src);
        cpu.step_destination_index(insn.a32(), sizeof(T));
    });
}

void SoftCPU::SCASB(const X86::Instruction& insn)
{
    do_scas<u8>(*this, insn);
}

void SoftCPU::SCASD(const X86::Instruction& insn)
{
    do_scas<u32>(*this, insn);
}

void SoftCPU::SCASW(const X86::Instruction& insn)
{
    do_scas<u16>(*this, insn);
}

void SoftCPU::SETcc_RM8(const X86::Instruction& insn)
{
    insn.modrm().write8(*this, insn, evaluate_condition(insn.cc()));
}

void SoftCPU::SGDT(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::SHLD_RM16_reg16_CL(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_shld(*this, insn.modrm().read16(*this, insn), const_gpr16(insn.reg16()), cl()));
}

void SoftCPU::SHLD_RM16_reg16_imm8(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_shld(*this, insn.modrm().read16(*this, insn), const_gpr16(insn.reg16()), insn.imm8()));
}

void SoftCPU::SHLD_RM32_reg32_CL(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_shld(*this, insn.modrm().read32(*this, insn), const_gpr32(insn.reg32()), cl()));
}

void SoftCPU::SHLD_RM32_reg32_imm8(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_shld(*this, insn.modrm().read32(*this, insn), const_gpr32(insn.reg32()), insn.imm8()));
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(SHL, op_shl)

void SoftCPU::SHRD_RM16_reg16_CL(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_shrd(*this, insn.modrm().read16(*this, insn), const_gpr16(insn.reg16()), cl()));
}

void SoftCPU::SHRD_RM16_reg16_imm8(const X86::Instruction& insn)
{
    insn.modrm().write16(*this, insn, op_shrd(*this, insn.modrm().read16(*this, insn), const_gpr16(insn.reg16()), insn.imm8()));
}

void SoftCPU::SHRD_RM32_reg32_CL(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_shrd(*this, insn.modrm().read32(*this, insn), const_gpr32(insn.reg32()), cl()));
}

void SoftCPU::SHRD_RM32_reg32_imm8(const X86::Instruction& insn)
{
    insn.modrm().write32(*this, insn, op_shrd(*this, insn.modrm().read32(*this, insn), const_gpr32(insn.reg32()), insn.imm8()));
}

DEFINE_GENERIC_SHIFT_ROTATE_INSN_HANDLERS(SHR, op_shr)

void SoftCPU::SIDT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::SLDT_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::SMSW_RM16(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::STC(const X86::Instruction&)
{
    set_cf(true);
}

void SoftCPU::STD(const X86::Instruction&)
{
    set_df(true);
}

void SoftCPU::STI(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::STOSB(const X86::Instruction& insn)
{
    if (insn.has_rep_prefix() && !df()) {
        // Fast path for 8-bit forward memory fill.
        if (m_emulator.fast_fill_memory8({ es(), destination_index(insn.a32()) }, ecx(), al())) {
            if (insn.a32()) {
                set_edi((u32)(edi() + ecx()));
                set_ecx(0);
            } else {
                set_di((u16)(di() + cx()));
                set_cx(0);
            }
            return;
        }
    }

    do_once_or_repeat<false>(insn, [&] {
        write_memory8({ es(), destination_index(insn.a32()) }, al());
        step_destination_index(insn.a32(), 1);
    });
}

void SoftCPU::STOSD(const X86::Instruction& insn)
{
    if (insn.has_rep_prefix() && !df()) {
        // Fast path for 32-bit forward memory fill.
        if (m_emulator.fast_fill_memory32({ es(), destination_index(insn.a32()) }, ecx(), eax())) {
            if (insn.a32()) {
                set_edi((u32)(edi() + (ecx() * sizeof(u32))));
                set_ecx(0);
            } else {
                set_di((u16)(di() + (cx() * sizeof(u32))));
                set_cx(0);
            }
            return;
        }
    }

    do_once_or_repeat<false>(insn, [&] {
        write_memory32({ es(), destination_index(insn.a32()) }, eax());
        step_destination_index(insn.a32(), 4);
    });
}

void SoftCPU::STOSW(const X86::Instruction& insn)
{
    do_once_or_repeat<false>(insn, [&] {
        write_memory16({ es(), destination_index(insn.a32()) }, ax());
        step_destination_index(insn.a32(), 2);
    });
}

void SoftCPU::STR_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::UD0(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::UD1(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::UD2(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::VERR_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::VERW_RM16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::WAIT(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::WBINVD(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::XADD_RM16_reg16(const X86::Instruction& insn)
{
    auto dest = insn.modrm().read16(*this, insn);
    auto src = const_gpr16(insn.reg16());
    auto result = op_add(*this, dest, src);
    gpr16(insn.reg16()) = dest;
    insn.modrm().write16(*this, insn, result);
}

void SoftCPU::XADD_RM32_reg32(const X86::Instruction& insn)
{
    auto dest = insn.modrm().read32(*this, insn);
    auto src = const_gpr32(insn.reg32());
    auto result = op_add(*this, dest, src);
    gpr32(insn.reg32()) = dest;
    insn.modrm().write32(*this, insn, result);
}

void SoftCPU::XADD_RM8_reg8(const X86::Instruction& insn)
{
    auto dest = insn.modrm().read8(*this, insn);
    auto src = const_gpr8(insn.reg8());
    auto result = op_add(*this, dest, src);
    gpr8(insn.reg8()) = dest;
    insn.modrm().write8(*this, insn, result);
}

void SoftCPU::XCHG_AX_reg16(const X86::Instruction& insn)
{
    auto temp = gpr16(insn.reg16());
    gpr16(insn.reg16()) = ax();
    set_ax(temp);
}

void SoftCPU::XCHG_EAX_reg32(const X86::Instruction& insn)
{
    auto temp = gpr32(insn.reg32());
    gpr32(insn.reg32()) = eax();
    set_eax(temp);
}

void SoftCPU::XCHG_reg16_RM16(const X86::Instruction& insn)
{
    auto temp = insn.modrm().read16(*this, insn);
    insn.modrm().write16(*this, insn, const_gpr16(insn.reg16()));
    gpr16(insn.reg16()) = temp;
}

void SoftCPU::XCHG_reg32_RM32(const X86::Instruction& insn)
{
    auto temp = insn.modrm().read32(*this, insn);
    insn.modrm().write32(*this, insn, const_gpr32(insn.reg32()));
    gpr32(insn.reg32()) = temp;
}

void SoftCPU::XCHG_reg8_RM8(const X86::Instruction& insn)
{
    auto temp = insn.modrm().read8(*this, insn);
    insn.modrm().write8(*this, insn, const_gpr8(insn.reg8()));
    gpr8(insn.reg8()) = temp;
}

void SoftCPU::XLAT(const X86::Instruction& insn)
{
    u32 offset = (insn.a32() ? ebx() : bx()) + al();
    set_al(read_memory8({ segment(insn.segment_prefix().value_or(X86::SegmentRegister::DS)), offset }));
}

#define DEFINE_GENERIC_INSN_HANDLERS_PARTIAL(mnemonic, op, update_dest)                                                             \
    void SoftCPU::mnemonic##_AL_imm8(const X86::Instruction& insn) { generic_AL_imm8<update_dest>(op<u8>, insn); }                                      \
    void SoftCPU::mnemonic##_AX_imm16(const X86::Instruction& insn) { generic_AX_imm16<update_dest>(op<u16>, insn); }                                   \
    void SoftCPU::mnemonic##_EAX_imm32(const X86::Instruction& insn) { generic_EAX_imm32<update_dest>(op<u32>, insn); }                                 \
    void SoftCPU::mnemonic##_RM16_imm16(const X86::Instruction& insn) { generic_RM16_imm16<update_dest>(op<u16>, insn); }                               \
    void SoftCPU::mnemonic##_RM16_reg16(const X86::Instruction& insn) { generic_RM16_reg16<update_dest>(op<u16>, insn); } \
    void SoftCPU::mnemonic##_RM32_imm32(const X86::Instruction& insn) { generic_RM32_imm32<update_dest>(op<u32>, insn); }                               \
    void SoftCPU::mnemonic##_RM32_reg32(const X86::Instruction& insn) { generic_RM32_reg32<update_dest>(op<u32>, insn); } \
    void SoftCPU::mnemonic##_RM8_imm8(const X86::Instruction& insn) { generic_RM8_imm8<update_dest>(op<u8>, insn); }                                    \
    void SoftCPU::mnemonic##_RM8_reg8(const X86::Instruction& insn) { generic_RM8_reg8<update_dest>(op<u8>, insn); }

#define DEFINE_GENERIC_INSN_HANDLERS(mnemonic, op, update_dest)                                                                     \
    DEFINE_GENERIC_INSN_HANDLERS_PARTIAL(mnemonic, op, update_dest)                                                                 \
    void SoftCPU::mnemonic##_RM16_imm8(const X86::Instruction& insn) { generic_RM16_imm8<update_dest>(op<u16>, insn); }                                 \
    void SoftCPU::mnemonic##_RM32_imm8(const X86::Instruction& insn) { generic_RM32_imm8<update_dest>(op<u32>, insn); }                                 \
    void SoftCPU::mnemonic##_reg16_RM16(const X86::Instruction& insn) { generic_reg16_RM16<update_dest>(op<u16>, insn); } \
    void SoftCPU::mnemonic##_reg32_RM32(const X86::Instruction& insn) { generic_reg32_RM32<update_dest>(op<u32>, insn); } \
    void SoftCPU::mnemonic##_reg8_RM8(const X86::Instruction& insn) { generic_reg8_RM8<update_dest>(op<u8>, insn); }

DEFINE_GENERIC_INSN_HANDLERS(XOR, op_xor, true)
DEFINE_GENERIC_INSN_HANDLERS(OR, op_or, true)
DEFINE_GENERIC_INSN_HANDLERS(ADD, op_add, true)
DEFINE_GENERIC_INSN_HANDLERS(ADC, op_adc, true)
DEFINE_GENERIC_INSN_HANDLERS(SUB, op_sub, true)
DEFINE_GENERIC_INSN_HANDLERS(SBB, op_sbb, true)
DEFINE_GENERIC_INSN_HANDLERS(AND, op_and, true)
DEFINE_GENERIC_INSN_HANDLERS(CMP, op_sub, false)
DEFINE_GENERIC_INSN_HANDLERS_PARTIAL(TEST, op_and, false)

void SoftCPU::MOVQ_mm1_mm2m64(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::EMMS(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::MOVQ_mm1_m64_mm2(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xC0(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xC1_16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xC1_32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD0(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD1_16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD1_32(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD2(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD3_16(const X86::Instruction&) { TODO_INSN(); }
void SoftCPU::wrap_0xD3_32(const X86::Instruction&) { TODO_INSN(); }

void SoftCPU::update_code_cache()
{
    if (auto* mem = m_emulator.find_memory<u8>(m_eip); mem && mem->type == BIOSEmulatorMemory::Type::Code) {
        m_cached_code = mem;
    } else {
        // TODO: halt
        VERIFY_NOT_REACHED();
    }
}

void BIOSEmulator::exec(BIOSEmulatorMemory& code, RegisterState& regs)
{
    add_memory(code);
    (void)regs;
    remove_memory(code);
}

BIOS::BIOS()
    : m_bios_rom(map_bios())
    , m_bios_ivt(MM.allocate_kernel_region(PhysicalAddress(0x0), page_round_up(0x400), {}, Region::Access::Read))
    , m_bios(m_bios_rom, BIOSEmulatorMemory::Type::Code)
    , m_ivt(FlatPtr(0x0), FlatPtr(0x400), m_bios_ivt->vaddr().as_ptr(), BIOSEmulatorMemory::Type::DataReadOnly)
    , m_emulator(new BIOSEmulator(m_ivt, m_bios))
{
}

BIOS::~BIOS()
{
    delete m_emulator;
}

void BIOS::call(u8 interrupt_number, RegisterState& regs)
{
    u8 code[]{ 0xcd, interrupt_number }; // int $interrupt_number
    FlatPtr code_addr = page_round_up(m_bios.end); // Place it after the bios rom
    BIOSEmulatorMemory code_mem(code_addr, FlatPtr(code_addr + sizeof(code)), &code[0], BIOSEmulatorMemory::Type::Code);

    auto original_eip = regs.eip;
    regs.eip = code_addr;

    m_emulator->exec(code_mem, regs);

    regs.eip = original_eip;
}

}
