/*
 * Copyright (c) 2021, Sahan Fernando <sahan.h.fernando@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/VirtIO/VirtIOGPU.h>
#include <LibC/sys/ioctl_numbers.h>

#define DEVICE_EVENTS_READ 0x0
#define DEVICE_EVENTS_CLEAR 0x4
#define DEVICE_NUM_SCANOUTS 0x8

namespace Kernel {

unsigned VirtIOGPU::next_device_id = 0;

VirtIOGPU::VirtIOGPU(PCI::Address address)
    : BlockDevice(226, next_device_id++)
    , VirtIODevice(address, "VirtIOGPU")
    , m_scratch_space(MM.allocate_contiguous_kernel_region(8 * PAGE_SIZE, "VirtGPU Scratch Space", Region::Access::Read | Region::Access::Write))
{
    VERIFY(!!m_scratch_space);
    dbgln("Gonna configure now");
    if (auto cfg = get_config(ConfigurationType::Device)) {
        bool success = negotiate_features([&](u64 supported_features) {
            u64 negotiated = 0;
            if (is_feature_set(supported_features, VIRTIO_GPU_F_VIRGL))
                dbgln("VirtIOGPU: VIRGL is not yet supported!");
            if (is_feature_set(supported_features, VIRTIO_GPU_F_EDID))
                dbgln("VirtIOGPU: EDID is not yet supported!");
            return negotiated;
        });
        if (success) {
            read_config_atomic([&]() {
                m_num_scanouts = config_read32(*cfg, 0x8);
            });
            dbgln("VirtIOGPU: num_scanouts: {}", m_num_scanouts);
            success = setup_queues(2); // CONTROLQ + CURSORQ
        }
        if (success) {
            finish_init();
        }
        // 1. Get display information using VIRTIO_GPU_CMD_GET_DISPLAY_INFO
        query_display_information();
        // 2. Create BUFFER using VIRTIO_GPU_CMD_RESOURCE_CREATE_2D
        create_framebuffer();
        // 3. Attach backing storage using  VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING
        attach_backing_storage();
        // 4. Use VIRTIO_GPU_CMD_SET_SCANOUT to link the framebuffer to a display scanout.
        attach_framebuffer_to_scanout();
        // 5. Render to your framebuffer memory.
        draw_pretty_pattern();
        // 6. Use VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D to update the host resource from guest memory.
        transfer_framebuffer_data_to_host(m_display_info.rect);
        // 7. Use VIRTIO_GPU_CMD_RESOURCE_FLUSH to flush the updated resource to the display.
        flush_displayed_image(m_display_info.rect);
    }
}

VirtIOGPU::~VirtIOGPU()
{
}

int VirtIOGPU::ioctl(FileDescription&, unsigned request, FlatPtr arg)
{
    REQUIRE_PROMISE(video);
    switch (request) {
    case FB_IOCTL_GET_SIZE_IN_BYTES: {
        auto* out = (size_t*)arg;
        size_t value = framebuffer_size_in_bytes();
        if (!copy_to_user(out, &value))
            return -EFAULT;
        return 0;
    }
    case FB_IOCTL_GET_RESOLUTION:
    case FB_IOCTL_SET_RESOLUTION: {
        auto* user_resolution = (FBResolution*)arg;
        FBResolution resolution;
        resolution.pitch = framebuffer_pitch();
        resolution.width = framebuffer_width();
        resolution.height = framebuffer_height();
        if (!copy_to_user(user_resolution, &resolution))
            return -EFAULT;
        return request == FB_IOCTL_GET_BUFFER ? 0 : -ENOTSUP;
    }
    case FB_IOCTL_FLUSH_BUFFER: {
        FBRect user_dirty_rect;
        if (!copy_from_user(&user_dirty_rect, (FBRect*)arg))
            return -EFAULT;
        VirtIOGPURect dirty_rect {
            .x = user_dirty_rect.x,
            .y = user_dirty_rect.y,
            .width = user_dirty_rect.width,
            .height = user_dirty_rect.height
        };
        transfer_framebuffer_data_to_host(m_display_info.rect);
        flush_displayed_image(dirty_rect);
        return 0;
    }
    default:
        return -EINVAL;
    };
}

KResultOr<Region*> VirtIOGPU::mmap(Process& process, FileDescription&, const Range& range, u64 offset, int prot, bool shared)
{
    REQUIRE_PROMISE(video);
    if (!shared)
        return ENODEV;
    if (offset != 0)
        return ENXIO;
    if (range.size() != page_round_up(framebuffer_size_in_bytes()))
        return EOVERFLOW;

    auto vmobject = m_framebuffer->vmobject().clone();
    if (!vmobject)
        return ENOMEM;
    return process.space().allocate_region_with_vmobject(
        range,
        vmobject.release_nonnull(),
        0,
        "VirtIOGPU Framebuffer",
        prot,
        shared);
}

bool VirtIOGPU::handle_device_config_change()
{
    return false;
}

void VirtIOGPU::handle_queue_update(u16 queue_index)
{
    dbgln_if(VIRTIO_DEBUG, "VirtIOConsole: Handle queue update");
    VERIFY(queue_index == CONTROLQ);

    auto& queue = get_queue(CONTROLQ);
    ScopedSpinLock queue_lock(queue.lock());
    queue.discard_used_buffers();
    m_has_outstanding_request = false;
}

u32 VirtIOGPU::get_pending_events()
{
    return config_read32(*m_device_configuration, DEVICE_EVENTS_READ);
}

void VirtIOGPU::clear_pending_events(u32 event_bitmask)
{
    config_write32(*m_device_configuration, DEVICE_EVENTS_READ, event_bitmask);
}

void VirtIOGPU::query_display_information()
{
    auto& request = *reinterpret_cast<VirtIOGPUCtrlHeader*>(m_scratch_space->vaddr().as_ptr());
    auto& response = *reinterpret_cast<VirtIOGPURespDisplayInfo*>((m_scratch_space->vaddr().offset(sizeof(request)).as_ptr()));

    populate_virtio_gpu_request_header(request, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_GET_DISPLAY_INFO, VIRTIO_GPU_FLAG_FENCE);

    synchronous_virtio_gpu_command(start_of_scratch_space(), sizeof(request), sizeof(response));

    for (size_t i = 0; i < VIRTIO_GPU_MAX_SCANOUTS; ++i) {
        auto& scanout = response.scanout_modes[i];
        if (!scanout.enabled)
            continue;
        dbgln("Scanout {}: x: {}, y: {}, width: {}, height: {}", i, scanout.rect.x, scanout.rect.y, scanout.rect.width, scanout.rect.height);
        m_display_info = scanout;
        m_chosen_scanout = i;
    }
    m_display_info.rect.width = 1024;
    m_display_info.rect.height = 768;
}

void VirtIOGPU::create_framebuffer()
{
    auto& request = *reinterpret_cast<VirtIOGPUResourceCreate2D*>(m_scratch_space->vaddr().as_ptr());
    auto& response = *reinterpret_cast<VirtIOGPUCtrlHeader*>((m_scratch_space->vaddr().offset(sizeof(request)).as_ptr()));

    populate_virtio_gpu_request_header(request.header, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_RESOURCE_CREATE_2D, VIRTIO_GPU_FLAG_FENCE);

    request.resource_id = m_framebuffer_id = 1;
    request.width = m_display_info.rect.width;
    request.height = m_display_info.rect.height;
    request.format = static_cast<u32>(VirtIOGPUFormats::VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM);

    synchronous_virtio_gpu_command(start_of_scratch_space(), sizeof(request), sizeof(response));

    VERIFY(response.type == static_cast<u32>(VirtIOGPUCtrlType::VIRTIO_GPU_RESP_OK_NODATA));
    dbgln("VirtIOGPU: Allocated resource id");
}

void VirtIOGPU::attach_backing_storage()
{
    // Allocate backing region
    size_t buffer_length = framebuffer_size_in_bytes();
    m_framebuffer = MM.allocate_kernel_region(page_round_up(buffer_length), "VirtGPU FrameBuffer", Region::Access::Read | Region::Access::Write, AllocationStrategy::AllocateNow);
    VERIFY(!!m_framebuffer);
    size_t num_mem_regions = page_round_up(buffer_length) / PAGE_SIZE;
    VERIFY(num_mem_regions == m_framebuffer->vmobject().page_count());

    // Send request
    auto& request = *reinterpret_cast<VirtIOGPUResourceAttachBacking*>(m_scratch_space->vaddr().as_ptr());
    const size_t header_block_size = sizeof(request) + num_mem_regions * sizeof(VirtIOGPUMemEntry);
    auto& response = *reinterpret_cast<VirtIOGPUCtrlHeader*>((m_scratch_space->vaddr().offset(header_block_size).as_ptr()));

    populate_virtio_gpu_request_header(request.header, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING, VIRTIO_GPU_FLAG_FENCE);
    request.resource_id = m_framebuffer_id = 1;
    request.num_entries = num_mem_regions;
    for (size_t i = 0; i < num_mem_regions; ++i) {
        request.entries[i].address = m_framebuffer->physical_page(i)->paddr().get();
        request.entries[i].length = min((size_t)4096, buffer_length - (i * PAGE_SIZE));
    }

    synchronous_virtio_gpu_command(start_of_scratch_space(), header_block_size, sizeof(response));

    VERIFY(response.type == static_cast<u32>(VirtIOGPUCtrlType::VIRTIO_GPU_RESP_OK_NODATA));
    dbgln("VirtIOGPU: Allocated backing storage");
}

size_t VirtIOGPU::framebuffer_size_in_bytes() const
{
    return m_display_info.rect.width * m_display_info.rect.height * sizeof(u32);
}

void VirtIOGPU::attach_framebuffer_to_scanout()
{
    auto& request = *reinterpret_cast<VirtIOGPUSetScanOut*>(m_scratch_space->vaddr().as_ptr());
    auto& response = *reinterpret_cast<VirtIOGPUCtrlHeader*>((m_scratch_space->vaddr().offset(sizeof(request)).as_ptr()));

    populate_virtio_gpu_request_header(request.header, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_SET_SCANOUT, VIRTIO_GPU_FLAG_FENCE);
    request.resource_id = m_framebuffer_id;
    request.scanout_id = m_chosen_scanout.value();
    request.rect = m_display_info.rect;

    synchronous_virtio_gpu_command(start_of_scratch_space(), sizeof(request), sizeof(response));

    VERIFY(response.type == static_cast<u32>(VirtIOGPUCtrlType::VIRTIO_GPU_RESP_OK_NODATA));
    dbgln("VirtIOGPU: Set backing scanout");
}

void VirtIOGPU::draw_pretty_pattern()
{
    size_t width = m_display_info.rect.width;
    size_t height = m_display_info.rect.height;
    size_t num_pixels = width * height;
    u8* data = m_framebuffer->vaddr().as_ptr();
    dbgln("Going to draw the pattern");
    // Clear the background to white
    for (size_t i = 0; i < 4 * num_pixels; ++i) {
        data[i] = 0xff;
    }
    // Circle drawing algorithm for red center
    auto draw_red_pixel_mirrored = [&](int x, int y) {
        VERIFY(0 <= x && x <= 200);
        VERIFY(0 <= y && y <= 200);
        for (auto i1 = 0; i1 < 2; ++i1) {
            auto nx = i1 ? y : x;
            auto ny = i1 ? x : y;
            for (auto i2 = 0; i2 < 2; ++i2) {
                for (auto i3 = 0; i3 < 2; ++i3) {
                    auto i = width * (ny + (height / 2)) + nx + (width / 2);
                    data[(4 * i) + 0] = 0;
                    data[(4 * i) + 1] = 0;
                    data[(4 * i) + 2] = 0xff;
                    data[(4 * i) + 3] = 0xff;
                    ny *= -1;
                }
                nx *= -1;
            }
        }
    };

    for (int y = 0; y < 200; ++y) {
        for (int x = 0; x * x + y * y < (150 * 150); ++x) {
            draw_red_pixel_mirrored(x, y);
        }
    }
    dbgln("Finish drawing the pattern");
}

void VirtIOGPU::transfer_framebuffer_data_to_host(VirtIOGPURect dirty_rect)
{
    auto& request = *reinterpret_cast<VirtIOGPUTransferToHost2D*>(m_scratch_space->vaddr().as_ptr());
    auto& response = *reinterpret_cast<VirtIOGPUCtrlHeader*>((m_scratch_space->vaddr().offset(sizeof(request)).as_ptr()));

    populate_virtio_gpu_request_header(request.header, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D, VIRTIO_GPU_FLAG_FENCE);
    request.offset = 0;
    request.resource_id = m_framebuffer_id;
    request.rect = dirty_rect;

    synchronous_virtio_gpu_command(start_of_scratch_space(), sizeof(request), sizeof(response));

    VERIFY(response.type == static_cast<u32>(VirtIOGPUCtrlType::VIRTIO_GPU_RESP_OK_NODATA));
}

void VirtIOGPU::flush_displayed_image(VirtIOGPURect dirty_rect)
{
    auto& request = *reinterpret_cast<VirtIOGPUResourceFlush*>(m_scratch_space->vaddr().as_ptr());
    auto& response = *reinterpret_cast<VirtIOGPUCtrlHeader*>((m_scratch_space->vaddr().offset(sizeof(request)).as_ptr()));

    populate_virtio_gpu_request_header(request.header, VirtIOGPUCtrlType::VIRTIO_GPU_CMD_RESOURCE_FLUSH, VIRTIO_GPU_FLAG_FENCE);
    request.resource_id = m_framebuffer_id;
    request.rect = dirty_rect;

    synchronous_virtio_gpu_command(start_of_scratch_space(), sizeof(request), sizeof(response));

    VERIFY(response.type == static_cast<u32>(VirtIOGPUCtrlType::VIRTIO_GPU_RESP_OK_NODATA));
}

void VirtIOGPU::synchronous_virtio_gpu_command(PhysicalAddress buffer_start, size_t request_size, size_t response_size)
{
    m_has_outstanding_request = true;
    auto& queue = get_queue(CONTROLQ);
    {
        ScopedSpinLock lock(queue.lock());
        VirtIOQueueChain chain { queue };
        chain.add_buffer_to_chain(buffer_start, request_size, BufferType::DeviceReadable);
        chain.add_buffer_to_chain(buffer_start.offset(request_size), response_size, BufferType::DeviceWritable);
        supply_chain_and_notify(CONTROLQ, chain);
    }
    full_memory_barrier();
    while (m_has_outstanding_request.load())
        Scheduler::yield();
}

void VirtIOGPU::populate_virtio_gpu_request_header(VirtIOGPUCtrlHeader& header, VirtIOGPUCtrlType ctrl_type, u32 flags)
{
    header.type = static_cast<u32>(ctrl_type);
    header.flags = flags;
    header.fence_id = 0;
    header.context_id = 0;
    header.padding = 0;
}

}
