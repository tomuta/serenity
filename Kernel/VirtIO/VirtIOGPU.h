/*
 * Copyright (c) 2021, Sahan Fernando <sahan.h.fernando@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <Kernel/Devices/BlockDevice.h>
#include <Kernel/VirtIO/VirtIO.h>
#include <Kernel/VirtIO/VirtIOQueue.h>

#define VIRTIO_GPU_F_VIRGL (1 << 0)
#define VIRTIO_GPU_F_EDID (1 << 1)

#define VIRTIO_GPU_FLAG_FENCE (1 << 0)

#define VIRTIO_GPU_MAX_SCANOUTS 16

#define CONTROLQ 0
#define CURSORQ 1

namespace Kernel {

enum class VirtIOGPUCtrlType : u32 {
    /* 2d commands */
    VIRTIO_GPU_CMD_GET_DISPLAY_INFO = 0x0100,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_2D,
    VIRTIO_GPU_CMD_RESOURCE_UNREF,
    VIRTIO_GPU_CMD_SET_SCANOUT,
    VIRTIO_GPU_CMD_RESOURCE_FLUSH,
    VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D,
    VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING,
    VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING,
    VIRTIO_GPU_CMD_GET_CAPSET_INFO,
    VIRTIO_GPU_CMD_GET_CAPSET,
    VIRTIO_GPU_CMD_GET_EDID,

    /* cursor commands */
    VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
    VIRTIO_GPU_CMD_MOVE_CURSOR,

    /* success responses */
    VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
    VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET,
    VIRTIO_GPU_RESP_OK_EDID,

    /* error responses */
    VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
    VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,
};

struct VirtIOGPUCtrlHeader {
    u32 type;
    u32 flags;
    u64 fence_id;
    u32 context_id;
    u32 padding;
};

struct VirtIOGPURect {
    u32 x;
    u32 y;
    u32 width;
    u32 height;
};

struct VirtIOGPURespDisplayInfo {
    VirtIOGPUCtrlHeader header;
    struct VirtIOGPUDisplayOne {
        VirtIOGPURect rect;
        u32 enabled;
        u32 flags;
    } scanout_modes[VIRTIO_GPU_MAX_SCANOUTS];
};

enum class VirtIOGPUFormats : u32 {
    VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM = 1,
    VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM = 2,
    VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM = 3,
    VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM = 4,

    VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM = 67,
    VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM = 68,

    VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM = 121,
    VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM = 134,
};

struct VirtIOGPUResourceCreate2D {
    VirtIOGPUCtrlHeader header;
    u32 resource_id;
    u32 format;
    u32 width;
    u32 height;
};

struct VirtIOGPUSetScanOut {
    VirtIOGPUCtrlHeader header;
    VirtIOGPURect rect;
    u32 scanout_id;
    u32 resource_id;
};

struct VirtIOGPUMemEntry {
    u64 address;
    u32 length;
    u32 padding;
};

struct VirtIOGPUResourceAttachBacking {
    VirtIOGPUCtrlHeader header;
    u32 resource_id;
    u32 num_entries;
    // TODO: Better way of doing this
    VirtIOGPUMemEntry entries[0];
};

struct VirtIOGPUTransferToHost2D {
    VirtIOGPUCtrlHeader header;
    VirtIOGPURect rect;
    u64 offset;
    u32 resource_id;
    u32 padding;
};

struct VirtIOGPUResourceFlush {
    VirtIOGPUCtrlHeader header;
    VirtIOGPURect rect;
    u32 resource_id;
    u32 padding;
};

class VirtIOGPU final : public BlockDevice
    , public VirtIODevice {
public:
    VirtIOGPU(PCI::Address);
    virtual ~VirtIOGPU() override;

    enum class DriverState : u32 {
        Initializing,
        ResizingScanout,
        Normal,
    };

private:
    virtual const char* class_name() const override { return m_class_name.characters(); }

    virtual int ioctl(FileDescription&, unsigned request, FlatPtr arg) override;
    virtual KResultOr<Region*> mmap(Process&, FileDescription&, const Range&, u64 offset, int prot, bool shared) override;
    virtual bool can_read(const FileDescription&, size_t) const override { return true; }
    virtual KResultOr<size_t> read(FileDescription&, u64, UserOrKernelBuffer&, size_t) override { return EINVAL; }
    virtual bool can_write(const FileDescription&, size_t) const override { return true; }
    virtual KResultOr<size_t> write(FileDescription&, u64, const UserOrKernelBuffer&, size_t) override { return EINVAL; };
    virtual void start_request(AsyncBlockDeviceRequest& request) override { request.complete(AsyncDeviceRequest::Failure); }

    virtual mode_t required_mode() const override { return 0666; }

    virtual bool handle_device_config_change() override;
    virtual String device_name() const override { return String::formatted("fb{}", minor()); }
    virtual void handle_queue_update(u16 queue_index) override;

    size_t framebuffer_width() { return m_display_info.rect.width; }
    size_t framebuffer_height() { return m_display_info.rect.height; }
    size_t framebuffer_pitch() { return m_display_info.rect.width * 4; }
    size_t framebuffer_size_in_bytes() const;

    u32 get_pending_events();
    void clear_pending_events(u32 event_bitmask);

    PhysicalAddress start_of_scratch_space() const { return m_scratch_space->physical_page(0)->paddr(); }
    void synchronous_virtio_gpu_command(PhysicalAddress buffer_start, size_t request_size, size_t response_size);
    void populate_virtio_gpu_request_header(VirtIOGPUCtrlHeader& header, VirtIOGPUCtrlType ctrl_type, u32 flags = 0);

    void query_display_information();
    void create_framebuffer();
    void attach_backing_storage();
    void attach_framebuffer_to_scanout();
    void draw_pretty_pattern();
    void transfer_framebuffer_data_to_host(VirtIOGPURect rect);
    void flush_displayed_image(VirtIOGPURect dirty_rect);

    VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne m_display_info;
    Optional<size_t> m_chosen_scanout {};
    u32 m_framebuffer_id { 0 };
    Configuration* m_device_configuration { nullptr };
    size_t m_num_scanouts {};
    OwnPtr<Region> m_framebuffer;
    DriverState m_current_state;

    // Synchronous commands
    Atomic<bool> m_has_outstanding_request { false };
    OwnPtr<Region> m_scratch_space;

    static unsigned next_device_id;
};

}
