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

class VirtIOGPU;

class VirtIOGPUFrameBuffer final : public BlockDevice {
    friend class VirtIOGPU;

public:
    VirtIOGPUFrameBuffer(VirtIOGPU&, u32);

    virtual const char* class_name() const override { return "VirtIOGPUFrameBuffer"; }

    virtual void start_request(AsyncBlockDeviceRequest& request) override { request.complete(AsyncDeviceRequest::Failure); }

    // TODO: adopt_ref?
    VMObject& framebuffer_vm_object() { return m_framebuffer->vmobject(); }

    size_t framebuffer_width();
    size_t framebuffer_height();
    size_t framebuffer_pitch();

    virtual int ioctl(FileDescription&, unsigned request, FlatPtr arg) override;
    virtual KResultOr<Region*> mmap(Process&, FileDescription&, const Range&, u64 offset, int prot, bool shared) override;
    virtual bool can_read(const FileDescription&, size_t) const override { return true; }
    virtual KResultOr<size_t> read(FileDescription&, u64, UserOrKernelBuffer&, size_t) override { return EINVAL; }
    virtual bool can_write(const FileDescription&, size_t) const override { return true; }
    virtual KResultOr<size_t> write(FileDescription&, u64, const UserOrKernelBuffer&, size_t) override { return EINVAL; };

private:
    inline VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne& display_info();
    inline const VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne& display_info() const;
    size_t framebuffer_size_in_bytes() const;
    inline Region& scratch_space();

    virtual String device_name() const override { return String::formatted("fb{}", m_device_id); }
    virtual mode_t required_mode() const override { return 0666; }

    void transfer_framebuffer_data_to_host(VirtIOGPURect rect);
    void flush_displayed_image(VirtIOGPURect dirty_rect);

    void create_framebuffer();
    void attach_backing_storage();
    void attach_framebuffer_to_scanout();
    void draw_pretty_pattern();

    VirtIOGPU& m_gpu;
    const u32 m_device_id;
    const u32 m_scanout;
    const u32 m_framebuffer_resource_id;
    OwnPtr<Region> m_framebuffer;

    static unsigned next_device_id;
};

class VirtIOGPU final : public Device
    , public VirtIODevice {
    friend class VirtIOGPUFrameBuffer;

public:
    VirtIOGPU(PCI::Address);
    virtual ~VirtIOGPU() override;

    RefPtr<VirtIOGPUFrameBuffer> framebuffer_device(u32 framebuffer_id) const
    {
        VERIFY(framebuffer_id < VIRTIO_GPU_MAX_SCANOUTS);
        return m_framebuffers[framebuffer_id];
    }
    enum class DriverState : u32 {
        Initializing,
        ResizingScanout,
        Normal,
    };

private:
    virtual const char* class_name() const override { return m_class_name.characters(); }

    virtual int ioctl(FileDescription&, unsigned, FlatPtr) override { return EINVAL; }
    virtual KResultOr<Region*> mmap(Process&, FileDescription&, const Range&, u64, int, bool) override { return EINVAL; }
    virtual bool can_read(const FileDescription&, size_t) const override { return false; }
    virtual KResultOr<size_t> read(FileDescription&, u64, UserOrKernelBuffer&, size_t) override { return EINVAL; }
    virtual bool can_write(const FileDescription&, size_t) const override { return false; }
    virtual KResultOr<size_t> write(FileDescription&, u64, const UserOrKernelBuffer&, size_t) override { return EINVAL; };

    virtual mode_t required_mode() const override { return 0666; }

    virtual bool handle_device_config_change() override;
    virtual String device_name() const override { return String::formatted("virtgpu{}", minor()); }
    virtual void handle_queue_update(u16 queue_index) override;

    u32 get_pending_events();
    void clear_pending_events(u32 event_bitmask);

    PhysicalAddress start_of_scratch_space() const { return m_scratch_space->physical_page(0)->paddr(); }
    void synchronous_virtio_gpu_command(PhysicalAddress buffer_start, size_t request_size, size_t response_size);
    void populate_virtio_gpu_request_header(VirtIOGPUCtrlHeader& header, VirtIOGPUCtrlType ctrl_type, u32 flags = 0);

    void query_display_information();

    VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne m_display_info[VIRTIO_GPU_MAX_SCANOUTS] {};
    RefPtr<VirtIOGPUFrameBuffer> m_framebuffers[VIRTIO_GPU_MAX_SCANOUTS];
    Configuration* m_device_configuration { nullptr };
    size_t m_num_scanouts { 0 };

    DriverState m_current_state;

    // Synchronous commands
    Atomic<bool> m_has_outstanding_request { false };
    OwnPtr<Region> m_scratch_space;

    static unsigned next_device_id;
};

inline VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne& VirtIOGPUFrameBuffer::display_info() { return m_gpu.m_display_info[m_scanout]; }
inline const VirtIOGPURespDisplayInfo::VirtIOGPUDisplayOne& VirtIOGPUFrameBuffer::display_info() const { return m_gpu.m_display_info[m_scanout]; }
inline Region& VirtIOGPUFrameBuffer::scratch_space() { return *m_gpu.m_scratch_space; }

}
