/*
 * Copyright (c) 2021, Sahan Fernando <sahan.h.fernando@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Graphics/Console/FramebufferConsole.h>
#include <Kernel/Graphics/GraphicsManagement.h>
#include <Kernel/Graphics/VirtIOGPU/VirtIOGraphicsAdapter.h>

namespace Kernel {

NonnullRefPtr<VirtIOGraphicsAdapter> VirtIOGraphicsAdapter::initialize(PCI::Address address)
{
    return adopt_ref(*new VirtIOGraphicsAdapter(address));
}

VirtIOGraphicsAdapter::VirtIOGraphicsAdapter(PCI::Address address)
    : m_base_address(address)
{
    VERIFY(GraphicsManagement::the().m_framebuffer_devices_allowed);
}

void VirtIOGraphicsAdapter::initialize_framebuffer_devices()
{
    VERIFY(m_framebuffer_device.is_null());
    m_framebuffer_gpu = adopt_ref(*new VirtIOGPU(m_base_address)).leak_ref();
    m_framebuffer_device = m_framebuffer_gpu->framebuffer_device(0);
    m_framebuffer_console = Kernel::Graphics::FramebufferConsole::initialize(m_framebuffer_device->framebuffer_vm_object(),
        m_framebuffer_device->framebuffer_width(),
        m_framebuffer_device->framebuffer_height(),
        m_framebuffer_device->framebuffer_pitch());
    // FIXME: This is a very wrong way to do this...
    GraphicsManagement::the().m_console = m_framebuffer_console;
}

void VirtIOGraphicsAdapter::enable_consoles()
{
    VERIFY(m_framebuffer_console);
    // if (m_framebuffer_device)
    //     m_framebuffer_device->dectivate_writes();
    m_framebuffer_console->enable();
}

void VirtIOGraphicsAdapter::disable_consoles()
{
    VERIFY(m_framebuffer_device);
    VERIFY(m_framebuffer_console);
    m_framebuffer_console->disable();
    // m_framebuffer_device->activate_writes();
}

bool VirtIOGraphicsAdapter::try_to_set_resolution(size_t, size_t, size_t)
{
    // TODO: Implement
    return false;
}

bool VirtIOGraphicsAdapter::set_y_offset(size_t, size_t)
{
    // TODO: Implement
    return false;
}

}
