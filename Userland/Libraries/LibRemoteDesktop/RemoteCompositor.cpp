/*
 * Copyright (c) 2021, the SerenityOS developers.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/Bitmap.h>
#include <LibRemoteDesktop/RemoteCompositor.h>

// Must be included after LibIPC/Forward.h
#include <LibIPC/Decoder.h>
#include <LibIPC/Encoder.h>

namespace IPC {

bool encode(Encoder& encoder, RemoteDesktop::Compositor::Window const& window)
{
    encoder << window.id << window.client_id << window.frame << window.geometry << window.opaque_rects << window.transparent_rects;
    return true;
}

bool decode(Decoder& decoder, RemoteDesktop::Compositor::Window& window)
{
    if (!decoder.decode(window.id))
        return false;
    if (!decoder.decode(window.client_id))
        return false;
    if (!decoder.decode(window.frame))
        return false;
    if (!decoder.decode(window.geometry))
        return false;
    if (!decoder.decode(window.opaque_rects))
        return false;
    if (!decoder.decode(window.transparent_rects))
        return false;
    return true;
}

bool encode(Encoder& encoder, RemoteDesktop::Compositor::WindowFrame const& window_frame)
{
    encoder << window_frame.top_bottom_bitmap_id << window_frame.left_right_bitmap_id;
    return true;
}

bool decode(Decoder& decoder, RemoteDesktop::Compositor::WindowFrame& window_frame)
{
    if (!decoder.decode(window_frame.top_bottom_bitmap_id))
        return false;
    if (!decoder.decode(window_frame.left_right_bitmap_id))
        return false;
    return true;
}

bool encode(Encoder& encoder, RemoteDesktop::Compositor::WindowGeometry const& window_geometry)
{
    encoder << window_geometry.render_rect << window_geometry.frame_rect << window_geometry.rect;
    return true;
}

bool decode(Decoder& decoder, RemoteDesktop::Compositor::WindowGeometry& window_geometry)
{
    if (!decoder.decode(window_geometry.render_rect))
        return false;
    if (!decoder.decode(window_geometry.frame_rect))
        return false;
    if (!decoder.decode(window_geometry.rect))
        return false;
    return true;
}

bool encode(Encoder& encoder, RemoteDesktop::Compositor::WindowDirtyRects const& window_dirty_rects)
{
    encoder << window_dirty_rects.id << window_dirty_rects.backing_bitmap_id << window_dirty_rects.is_windowserver_backing_bitmap << window_dirty_rects.backing_bitmap_sync_tag << window_dirty_rects.frame_left_right_bitmap_id << window_dirty_rects.frame_top_bottom_bitmap_id << window_dirty_rects.dirty_rects;
    return true;
}

bool decode(Decoder& decoder, RemoteDesktop::Compositor::WindowDirtyRects& window_dirty_rects)
{
    if (!decoder.decode(window_dirty_rects.id))
        return false;
    if (!decoder.decode(window_dirty_rects.backing_bitmap_id))
        return false;
    if (!decoder.decode(window_dirty_rects.is_windowserver_backing_bitmap))
        return false;
    if (!decoder.decode(window_dirty_rects.backing_bitmap_sync_tag))
        return false;
    if (!decoder.decode(window_dirty_rects.frame_left_right_bitmap_id))
        return false;
    if (!decoder.decode(window_dirty_rects.frame_top_bottom_bitmap_id))
        return false;
    if (!decoder.decode(window_dirty_rects.dirty_rects))
        return false;
    return true;
}

}
