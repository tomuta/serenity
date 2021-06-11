/*
 * Copyright (c) 2021, Sahan Fernando <sahan.h.fernando@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Graphics/VGACompatibleAdapter.h>
#include <Kernel/VirtIO/VirtIOGPU.h>

namespace Kernel {

class VirtIOGraphicsAdapter final : public GraphicsDevice {
    AK_MAKE_ETERNAL
public:
    static NonnullRefPtr<VirtIOGraphicsAdapter> initialize(PCI::Address);

    virtual bool framebuffer_devices_initialized() const override { return !m_framebuffer_device.is_null(); }

private:
    explicit VirtIOGraphicsAdapter(PCI::Address unnamed);

    virtual void initialize_framebuffer_devices() override;
    virtual Type type() const override { return Type::Raw; }

    virtual void enable_consoles() override;
    virtual void disable_consoles() override;

    virtual bool modesetting_capable() const override { return false; }
    virtual bool double_framebuffering_capable() const override { return false; }
    virtual bool try_to_set_resolution(size_t, size_t, size_t) override;
    virtual bool set_y_offset(size_t, size_t) override;

    PCI::Address m_base_address;
    RefPtr<VirtIOGPU> m_framebuffer_gpu;
    RefPtr<VirtIOGPUFrameBuffer> m_framebuffer_device;
    RefPtr<Graphics::Console> m_framebuffer_console {};
};

}
