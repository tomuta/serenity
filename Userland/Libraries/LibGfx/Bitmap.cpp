/*
 * Copyright (c) 2018-2021, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/Checked.h>
#include <AK/LexicalPath.h>
#include <AK/Memory.h>
#include <AK/MemoryStream.h>
#include <AK/Optional.h>
#include <AK/ScopeGuard.h>
#include <AK/String.h>
#include <LibGfx/BMPLoader.h>
#include <LibGfx/Bitmap.h>
#include <LibGfx/DDSLoader.h>
#include <LibGfx/GIFLoader.h>
#include <LibGfx/ICOLoader.h>
#include <LibGfx/JPGLoader.h>
#include <LibGfx/PBMLoader.h>
#include <LibGfx/PGMLoader.h>
#include <LibGfx/PNGLoader.h>
#include <LibGfx/PPMLoader.h>
#include <LibGfx/Remote/RemoteGfxServerConnection.h>
#include <LibGfx/ShareableBitmap.h>
#include <errno.h>
#include <stdio.h>
#include <sys/mman.h>

namespace Gfx {

struct BackingStore {
    void* data { nullptr };
    size_t pitch { 0 };
    size_t size_in_bytes { 0 };
};

size_t Bitmap::minimum_pitch(size_t physical_width, BitmapFormat format)
{
    size_t element_size;
    switch (determine_storage_format(format)) {
    case StorageFormat::Indexed8:
        element_size = 1;
        break;
    case StorageFormat::BGRx8888:
    case StorageFormat::BGRA8888:
    case StorageFormat::RGBA8888:
        element_size = 4;
        break;
    default:
        VERIFY_NOT_REACHED();
    }

    return physical_width * element_size;
}

static bool size_would_overflow(BitmapFormat format, const IntSize& size, int scale_factor)
{
    if (size.width() < 0 || size.height() < 0)
        return true;
    // This check is a bit arbitrary, but should protect us from most shenanigans:
    if (size.width() >= INT16_MAX || size.height() >= INT16_MAX || scale_factor < 1 || scale_factor > 4)
        return true;
    // In contrast, this check is absolutely necessary:
    size_t pitch = Bitmap::minimum_pitch(size.width() * scale_factor, format);
    return Checked<size_t>::multiplication_would_overflow(pitch, size.height() * scale_factor);
}

RefPtr<Bitmap> Bitmap::try_create(BitmapFormat format, const IntSize& size, int scale_factor)
{
    auto backing_store = Bitmap::try_allocate_backing_store(format, size, scale_factor);
    if (!backing_store.has_value())
        return nullptr;
    return adopt_ref(*new Bitmap(format, size, scale_factor, backing_store.value()));
}

RefPtr<Bitmap> Bitmap::try_create_shareable(BitmapFormat format, const IntSize& size, int scale_factor)
{
    if (size_would_overflow(format, size, scale_factor))
        return nullptr;

    const auto pitch = minimum_pitch(size.width() * scale_factor, format);
    const auto data_size = size_in_bytes(pitch, size.height() * scale_factor);

    auto buffer = Core::AnonymousBuffer::create_with_size(round_up_to_power_of_two(data_size, PAGE_SIZE));
    if (!buffer.is_valid())
        return nullptr;
    return Bitmap::try_create_with_anonymous_buffer(format, buffer, size, scale_factor, {});
}

Bitmap::Bitmap(BitmapFormat format, const IntSize& size, int scale_factor, const BackingStore& backing_store)
    : m_size(size)
    , m_scale(scale_factor)
    , m_data(backing_store.data)
    , m_pitch(backing_store.pitch)
    , m_format(format)
{
    VERIFY(!m_size.is_empty());
    VERIFY(!size_would_overflow(format, size, scale_factor));
    VERIFY(m_data);
    VERIFY(backing_store.size_in_bytes == size_in_bytes());
    allocate_palette_from_format(format, {});
    m_needs_munmap = true;
}

RefPtr<Bitmap> Bitmap::try_create_wrapper(BitmapFormat format, const IntSize& size, int scale_factor, size_t pitch, void* data)
{
    if (size_would_overflow(format, size, scale_factor))
        return nullptr;
    return adopt_ref(*new Bitmap(format, size, scale_factor, pitch, data));
}

RefPtr<Bitmap> Bitmap::try_load_from_file(String const& path, int scale_factor)
{
    if (scale_factor > 1 && path.starts_with("/res/")) {
        LexicalPath lexical_path { path };
        StringBuilder highdpi_icon_path;
        highdpi_icon_path.append(lexical_path.dirname());
        highdpi_icon_path.append('/');
        highdpi_icon_path.append(lexical_path.title());
        highdpi_icon_path.appendff("-{}x.", scale_factor);
        highdpi_icon_path.append(lexical_path.extension());

        RefPtr<Bitmap> bmp;
#define __ENUMERATE_IMAGE_FORMAT(Name, Ext)                    \
    if (path.ends_with(Ext, CaseSensitivity::CaseInsensitive)) \
        bmp = load_##Name(highdpi_icon_path.to_string());
        ENUMERATE_IMAGE_FORMATS
#undef __ENUMERATE_IMAGE_FORMAT
        if (bmp) {
            VERIFY(bmp->width() % scale_factor == 0);
            VERIFY(bmp->height() % scale_factor == 0);
            bmp->m_size.set_width(bmp->width() / scale_factor);
            bmp->m_size.set_height(bmp->height() / scale_factor);
            bmp->m_scale = scale_factor;
            return bmp;
        }
    }

#define __ENUMERATE_IMAGE_FORMAT(Name, Ext)                    \
    if (path.ends_with(Ext, CaseSensitivity::CaseInsensitive)) \
        return load_##Name(path);
    ENUMERATE_IMAGE_FORMATS
#undef __ENUMERATE_IMAGE_FORMAT

    return nullptr;
}

Bitmap::Bitmap(BitmapFormat format, const IntSize& size, int scale_factor, size_t pitch, void* data)
    : m_size(size)
    , m_scale(scale_factor)
    , m_data(data)
    , m_pitch(pitch)
    , m_format(format)
{
    VERIFY(pitch >= minimum_pitch(size.width() * scale_factor, format));
    VERIFY(!size_would_overflow(format, size, scale_factor));
    // FIXME: assert that `data` is actually long enough!

    allocate_palette_from_format(format, {});
}

static bool check_size(const IntSize& size, int scale_factor, BitmapFormat format, unsigned actual_size)
{
    // FIXME: Code duplication of size_in_bytes() and m_pitch
    unsigned expected_size_min = Bitmap::minimum_pitch(size.width() * scale_factor, format) * size.height() * scale_factor;
    unsigned expected_size_max = round_up_to_power_of_two(expected_size_min, PAGE_SIZE);
    if (expected_size_min > actual_size || actual_size > expected_size_max) {
        // Getting here is most likely an error.
        dbgln("Constructing a shared bitmap for format {} and size {} @ {}x, which demands {} bytes, which rounds up to at most {}.",
            static_cast<int>(format),
            size,
            scale_factor,
            expected_size_min,
            expected_size_max);

        dbgln("However, we were given {} bytes, which is outside this range?! Refusing cowardly.", actual_size);
        return false;
    }
    return true;
}

RefPtr<Bitmap> Bitmap::try_create_with_anonymous_buffer(BitmapFormat format, Core::AnonymousBuffer buffer, const IntSize& size, int scale_factor, const Vector<RGBA32>& palette)
{
    if (size_would_overflow(format, size, scale_factor))
        return nullptr;

    return adopt_ref(*new Bitmap(format, move(buffer), size, scale_factor, palette));
}

/// Read a bitmap as described by:
/// - actual size
/// - width
/// - height
/// - scale_factor
/// - format
/// - palette count
/// - palette data (= palette count * BGRA8888)
/// - image data (= actual size * u8)
RefPtr<Bitmap> Bitmap::try_create_from_serialized_byte_buffer(ByteBuffer&& buffer)
{
    InputMemoryStream stream { buffer };
    size_t actual_size;
    unsigned width;
    unsigned height;
    unsigned scale_factor;
    BitmapFormat format;
    unsigned palette_size;
    Vector<RGBA32> palette;

    auto read = [&]<typename T>(T& value) {
        if (stream.read({ &value, sizeof(T) }) != sizeof(T))
            return false;
        return true;
    };

    if (!read(actual_size) || !read(width) || !read(height) || !read(scale_factor) || !read(format) || !read(palette_size))
        return nullptr;

    if (format > BitmapFormat::BGRA8888 || format < BitmapFormat::Indexed1)
        return nullptr;

    if (!check_size({ width, height }, scale_factor, format, actual_size))
        return {};

    palette.ensure_capacity(palette_size);
    for (size_t i = 0; i < palette_size; ++i) {
        if (!read(palette[i]))
            return {};
    }

    if (stream.remaining() < actual_size)
        return {};

    auto data = stream.bytes().slice(stream.offset(), actual_size);

    auto bitmap = Bitmap::try_create(format, { width, height }, scale_factor);
    if (!bitmap)
        return {};

    bitmap->m_palette = new RGBA32[palette_size];
    memcpy(bitmap->m_palette, palette.data(), palette_size * sizeof(RGBA32));

    data.copy_to({ bitmap->scanline(0), bitmap->size_in_bytes() });

    return bitmap;
}

ByteBuffer Bitmap::serialize_to_byte_buffer() const
{
    auto buffer = ByteBuffer::create_uninitialized(sizeof(size_t) + 4 * sizeof(unsigned) + sizeof(BitmapFormat) + sizeof(RGBA32) * palette_size(m_format) + size_in_bytes());
    OutputMemoryStream stream { buffer };

    auto write = [&]<typename T>(T value) {
        if (stream.write({ &value, sizeof(T) }) != sizeof(T))
            return false;
        return true;
    };

    auto palette = palette_to_vector();

    if (!write(size_in_bytes()) || !write((unsigned)size().width()) || !write((unsigned)size().height()) || !write((unsigned)scale()) || !write(m_format) || !write((unsigned)palette.size()))
        return {};

    for (auto& p : palette) {
        if (!write(p))
            return {};
    }

    auto size = size_in_bytes();
    VERIFY(stream.remaining() == size);
    if (stream.write({ scanline(0), size }) != size)
        return {};

    return buffer;
}

Bitmap::Bitmap(BitmapFormat format, Core::AnonymousBuffer buffer, const IntSize& size, int scale_factor, const Vector<RGBA32>& palette)
    : m_size(size)
    , m_scale(scale_factor)
    , m_data(buffer.data<void>())
    , m_pitch(minimum_pitch(size.width() * scale_factor, format))
    , m_format(format)
    , m_buffer(move(buffer))
{
    VERIFY(!is_indexed() || !palette.is_empty());
    VERIFY(!size_would_overflow(format, size, scale_factor));

    if (is_indexed(m_format))
        allocate_palette_from_format(m_format, palette);
}

RefPtr<Gfx::Bitmap> Bitmap::clone() const
{
    auto new_bitmap = Bitmap::try_create(format(), size(), scale());

    if (!new_bitmap)
        return nullptr;

    VERIFY(size_in_bytes() == new_bitmap->size_in_bytes());
    memcpy(new_bitmap->scanline(0), scanline(0), size_in_bytes());

    return new_bitmap;
}

RefPtr<Gfx::Bitmap> Bitmap::rotated(Gfx::RotationDirection rotation_direction) const
{
    auto new_bitmap = Gfx::Bitmap::try_create(this->format(), { height(), width() }, scale());
    if (!new_bitmap)
        return nullptr;

    auto w = this->physical_width();
    auto h = this->physical_height();
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            Color color;
            if (rotation_direction == Gfx::RotationDirection::CounterClockwise)
                color = this->get_pixel(w - i - 1, j);
            else
                color = this->get_pixel(i, h - j - 1);

            new_bitmap->set_pixel(j, i, color);
        }
    }

    return new_bitmap;
}

RefPtr<Gfx::Bitmap> Bitmap::flipped(Gfx::Orientation orientation) const
{
    auto new_bitmap = Gfx::Bitmap::try_create(this->format(), { width(), height() }, scale());
    if (!new_bitmap)
        return nullptr;

    auto w = this->physical_width();
    auto h = this->physical_height();
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            Color color = this->get_pixel(i, j);
            if (orientation == Orientation::Vertical)
                new_bitmap->set_pixel(i, h - j - 1, color);
            else
                new_bitmap->set_pixel(w - i - 1, j, color);
        }
    }

    return new_bitmap;
}

RefPtr<Gfx::Bitmap> Bitmap::scaled(int sx, int sy) const
{
    VERIFY(sx >= 0 && sy >= 0);
    if (sx == 1 && sy == 1)
        return this;

    auto new_bitmap = Gfx::Bitmap::try_create(format(), { width() * sx, height() * sy }, scale());
    if (!new_bitmap)
        return nullptr;

    auto old_width = physical_width();
    auto old_height = physical_height();

    for (int y = 0; y < old_height; y++) {
        for (int x = 0; x < old_width; x++) {
            auto color = get_pixel(x, y);

            auto base_x = x * sx;
            auto base_y = y * sy;
            for (int new_y = base_y; new_y < base_y + sy; new_y++) {
                for (int new_x = base_x; new_x < base_x + sx; new_x++) {
                    new_bitmap->set_pixel(new_x, new_y, color);
                }
            }
        }
    }

    return new_bitmap;
}

// http://fourier.eng.hmc.edu/e161/lectures/resize/node3.html
RefPtr<Gfx::Bitmap> Bitmap::scaled(float sx, float sy) const
{
    VERIFY(sx >= 0.0f && sy >= 0.0f);
    if (floorf(sx) == sx && floorf(sy) == sy)
        return scaled(static_cast<int>(sx), static_cast<int>(sy));

    int scaled_width = (int)ceilf(sx * (float)width());
    int scaled_height = (int)ceilf(sy * (float)height());

    auto new_bitmap = Gfx::Bitmap::try_create(format(), { scaled_width, scaled_height }, scale());
    if (!new_bitmap)
        return nullptr;

    auto old_width = physical_width();
    auto old_height = physical_height();
    auto new_width = new_bitmap->physical_width();
    auto new_height = new_bitmap->physical_height();

    // The interpolation goes out of bounds on the bottom- and right-most edges.
    // We handle those in two specialized loops not only to make them faster, but
    // also to avoid four branch checks for every pixel.

    for (int y = 0; y < new_height - 1; y++) {
        for (int x = 0; x < new_width - 1; x++) {
            auto p = static_cast<float>(x) * static_cast<float>(old_width - 1) / static_cast<float>(new_width - 1);
            auto q = static_cast<float>(y) * static_cast<float>(old_height - 1) / static_cast<float>(new_height - 1);

            int i = floorf(p);
            int j = floorf(q);
            float u = p - static_cast<float>(i);
            float v = q - static_cast<float>(j);

            auto a = get_pixel(i, j);
            auto b = get_pixel(i + 1, j);
            auto c = get_pixel(i, j + 1);
            auto d = get_pixel(i + 1, j + 1);

            auto e = a.interpolate(b, u);
            auto f = c.interpolate(d, u);
            auto color = e.interpolate(f, v);
            new_bitmap->set_pixel(x, y, color);
        }
    }

    // Bottom strip (excluding last pixel)
    auto old_bottom_y = old_height - 1;
    auto new_bottom_y = new_height - 1;
    for (int x = 0; x < new_width - 1; x++) {
        auto p = static_cast<float>(x) * static_cast<float>(old_width - 1) / static_cast<float>(new_width - 1);

        int i = floorf(p);
        float u = p - static_cast<float>(i);

        auto a = get_pixel(i, old_bottom_y);
        auto b = get_pixel(i + 1, old_bottom_y);
        auto color = a.interpolate(b, u);
        new_bitmap->set_pixel(x, new_bottom_y, color);
    }

    // Right strip (excluding last pixel)
    auto old_right_x = old_width - 1;
    auto new_right_x = new_width - 1;
    for (int y = 0; y < new_height - 1; y++) {
        auto q = static_cast<float>(y) * static_cast<float>(old_height - 1) / static_cast<float>(new_height - 1);

        int j = floorf(q);
        float v = q - static_cast<float>(j);

        auto c = get_pixel(old_right_x, j);
        auto d = get_pixel(old_right_x, j + 1);

        auto color = c.interpolate(d, v);
        new_bitmap->set_pixel(new_right_x, y, color);
    }

    // Bottom-right pixel
    new_bitmap->set_pixel(new_width - 1, new_height - 1, get_pixel(physical_width() - 1, physical_height() - 1));

    return new_bitmap;
}

RefPtr<Gfx::Bitmap> Bitmap::cropped(Gfx::IntRect crop) const
{
    auto new_bitmap = Gfx::Bitmap::try_create(format(), { crop.width(), crop.height() }, 1);
    if (!new_bitmap)
        return nullptr;

    for (int y = 0; y < crop.height(); ++y) {
        for (int x = 0; x < crop.width(); ++x) {
            int global_x = x + crop.left();
            int global_y = y + crop.top();
            if (global_x >= physical_width() || global_y >= physical_height() || global_x < 0 || global_y < 0) {
                new_bitmap->set_pixel(x, y, Gfx::Color::Black);
            } else {
                new_bitmap->set_pixel(x, y, get_pixel(global_x, global_y));
            }
        }
    }
    return new_bitmap;
}

RefPtr<Bitmap> Bitmap::to_bitmap_backed_by_anonymous_buffer() const
{
    if (m_buffer.is_valid())
        return *this;
    auto buffer = Core::AnonymousBuffer::create_with_size(round_up_to_power_of_two(size_in_bytes(), PAGE_SIZE));
    if (!buffer.is_valid())
        return nullptr;
    auto bitmap = Bitmap::try_create_with_anonymous_buffer(m_format, move(buffer), size(), scale(), palette_to_vector());
    if (!bitmap)
        return nullptr;
    memcpy(bitmap->scanline(0), scanline(0), size_in_bytes());
    return bitmap;
}

Bitmap::~Bitmap()
{
    if (m_needs_munmap) {
        int rc = munmap(m_data, size_in_bytes());
        VERIFY(rc == 0);
    }
    m_data = nullptr;
    delete[] m_palette;

    if (m_remote_data)
        destroy_remote_data();
}

void Bitmap::set_mmap_name([[maybe_unused]] String const& name)
{
    VERIFY(m_needs_munmap);
#ifdef __serenity__
    ::set_mmap_name(m_data, size_in_bytes(), name.characters());
#endif
}

void Bitmap::fill(Color color)
{
    VERIFY(!is_indexed(m_format));
    for (int y = 0; y < physical_height(); ++y) {
        auto* scanline = this->scanline(y);
        fast_u32_fill(scanline, color.value(), physical_width());
    }
}

void Bitmap::set_volatile()
{
    if (m_volatile)
        return;
#ifdef __serenity__
    int rc = madvise(m_data, size_in_bytes(), MADV_SET_VOLATILE);
    if (rc < 0) {
        perror("madvise(MADV_SET_VOLATILE)");
        VERIFY_NOT_REACHED();
    }
#endif
    m_volatile = true;
}

[[nodiscard]] bool Bitmap::set_nonvolatile(bool& was_purged)
{
    if (!m_volatile) {
        was_purged = false;
        return true;
    }

#ifdef __serenity__
    int rc = madvise(m_data, size_in_bytes(), MADV_SET_NONVOLATILE);
    if (rc < 0) {
        if (errno == ENOMEM) {
            was_purged = true;
            return false;
        }
        perror("madvise(MADV_SET_NONVOLATILE)");
        VERIFY_NOT_REACHED();
    }
    was_purged = rc != 0;
#endif
    m_volatile = false;
    return true;
}

ShareableBitmap Bitmap::to_shareable_bitmap() const
{
    auto bitmap = to_bitmap_backed_by_anonymous_buffer();
    if (!bitmap)
        return {};
    return ShareableBitmap(*bitmap);
}

Optional<BackingStore> Bitmap::try_allocate_backing_store(BitmapFormat format, IntSize const& size, int scale_factor)
{
    if (size_would_overflow(format, size, scale_factor))
        return {};

    const auto pitch = minimum_pitch(size.width() * scale_factor, format);
    const auto data_size_in_bytes = size_in_bytes(pitch, size.height() * scale_factor);

    int map_flags = MAP_ANONYMOUS | MAP_PRIVATE;
#ifdef __serenity__
    map_flags |= MAP_PURGEABLE;
    void* data = mmap_with_name(nullptr, data_size_in_bytes, PROT_READ | PROT_WRITE, map_flags, 0, 0, String::formatted("GraphicsBitmap [{}]", size).characters());
#else
    void* data = mmap(nullptr, data_size_in_bytes, PROT_READ | PROT_WRITE, map_flags, 0, 0);
#endif
    if (data == MAP_FAILED) {
        perror("mmap");
        return {};
    }
    return { { data, pitch, data_size_in_bytes } };
}

void Bitmap::allocate_palette_from_format(BitmapFormat format, const Vector<RGBA32>& source_palette)
{
    size_t size = palette_size(format);
    if (size == 0)
        return;
    m_palette = new RGBA32[size];
    if (!source_palette.is_empty()) {
        VERIFY(source_palette.size() == size);
        memcpy(m_palette, source_palette.data(), size * sizeof(RGBA32));
    }
}

Vector<RGBA32> Bitmap::palette_to_vector() const
{
    Vector<RGBA32> vector;
    auto size = palette_size(m_format);
    vector.ensure_capacity(size);
    for (size_t i = 0; i < size; ++i)
        vector.unchecked_append(palette_color(i).value());
    return vector;
}

bool Bitmap::is_rect_equal(Gfx::IntRect const& rect, Gfx::Bitmap const& other_bitmap, Gfx::IntPoint const& other_location) const
{
    if (this == &other_bitmap)
        return true;
    auto intersected_rect = rect.intersected(this->rect());
    if (intersected_rect.is_empty())
        return false;
    if (!other_bitmap.rect().contains({ other_location, intersected_rect.size() }))
        return false;

    auto physical_rect = intersected_rect * scale();
    auto other_physical_location = other_location * other_bitmap.scale();
    if (bpp() < 8 || other_bitmap.bpp() != bpp() || other_bitmap.scale() != scale() || other_bitmap.format() != format()) {
        if (other_bitmap.scale() == scale()) {
            // Slow path, compare pixel by pixel
            for (int y = physical_rect.top(), other_y = other_physical_location.y(); y <= physical_rect.bottom(); y++, other_y++) {
                for (int x = physical_rect.left(), other_x = other_physical_location.x(); x <= physical_rect.right(); x++, other_x++) {
                    if (get_pixel(x, y) != other_bitmap.get_pixel(other_x, other_y))
                        return false;
                }
            }
        } else {
            dbgln("Bitmap::is_rect_equal with different scales is not yet implemented!");
            VERIFY_NOT_REACHED();
        }
        return true;
    }

    // Fast path, try to memcmp the data
    VERIFY(bpp() == other_bitmap.bpp());
    size_t x_offset = (physical_rect.left() * bpp()) / 8;
    size_t other_x_offset = (other_location.x() * bpp()) / 8;
    size_t line_size = (physical_rect.width() * bpp()) / 8;
    for (int y = physical_rect.top(), other_y = other_physical_location.y(); y <= physical_rect.bottom(); y++, other_y++) {
        auto* this_line = scanline_u8(y) + x_offset;
        auto* other_line = other_bitmap.scanline_u8(other_y) + other_x_offset;
        if (__builtin_memcmp(this_line, other_line, line_size) != 0)
            return false;
    }
    return true;
}

Bitmap::RemoteData::RemoteData([[maybe_unused]] RemoteGfx::RemoteGfxSession& session)
#ifdef __serenity__
    : session(session.make_weak_ptr<RemoteGfx::RemoteGfxSession>())
#endif
{
}

int Bitmap::enable_remote_painting(bool enable, [[maybe_unused]] bool allow_sending_content)
{
    if (enable) {
#ifdef __serenity__
        if (m_remote_data) {
            VERIFY(m_remote_data->bitmap_id > 0);
            return 0;
        }
        if (m_local_only)
            return 0;
        auto remote_gfx_session = RemoteGfx::RemoteGfxServerConnection::the().session();
        if (!remote_gfx_session)
            return 0;
        if (!m_remote_data)
            m_remote_data = adopt_own(*new RemoteData(*remote_gfx_session));
        auto& remote_data = *m_remote_data;
        VERIFY(remote_data.bitmap_id == 0);
        static RemoteGfx::BitmapId s_bitmap_id = 0;
        remote_data.bitmap_id = ++s_bitmap_id;
        VERIFY(remote_data.bitmap_id > 0);
        if (allow_sending_content)
            remote_data.needs_tiles = IntRect { 0, 0, ceil_div(width(), RemoteData::tile_size), ceil_div(height(), RemoteData::tile_size) };
        else
            remote_data.never_send_content = true;
        remote_gfx_session->connection().async_create_bitmap(remote_data.bitmap_id, format(), size(), scale());
        return remote_data.bitmap_id;
#endif
    }

    m_remote_data = nullptr;
    return 0;
}

void Bitmap::send_to_remote(IntRect const& send_rect)
{
    VERIFY(rect().contains(send_rect));
#ifdef __serenity__
    if (!m_remote_data)
        return;
    auto& remote_data = *m_remote_data;
    VERIFY(remote_data.bitmap_id > 0);
    if (auto* remote_gfx = remote_data.session.ptr()) {
        if (remote_data.never_send_content)
            return;
        if (!remote_data.bitmap_sent) {
            remote_data.bitmap_sent = Bitmap::try_create(format(), size(), scale());
            if (remote_data.bitmap_sent)
                remote_data.bitmap_sent->disable_remote_painting();
        }
        IntRect send_tiles { send_rect.left() / RemoteData::tile_size, send_rect.top() / RemoteData::tile_size, ceil_div(send_rect.width(), RemoteData::tile_size), ceil_div(send_rect.height(), RemoteData::tile_size) };
        if (remote_data.bitmap_sent) {
            // Create a diff for the bitmap and apply the diff to bitmap_sent to keep it in sync with the remote client
            Gfx::DisjointRectSet rects_to_send;
            rects_to_send.add_many_transformed(remote_data.needs_tiles.rects(), [&](auto& rect) {
                auto transformed_rect = rect.intersected(send_tiles);
                if (!transformed_rect.is_empty())
                    transformed_rect *= RemoteData::tile_size;
                return transformed_rect;
            });
            auto bitmap_diff = RemoteGfx::BitmapDiff::create(remote_data.bitmap_id, *remote_data.bitmap_sent, *this, rects_to_send);
            if (!bitmap_diff.is_empty()) {
                remote_gfx->connection().async_apply_bitmap_diff(remote_data.bitmap_id, bitmap_diff);
                // Now apply the diff to bitmap_sent just like the remote client would do
                bitmap_diff.apply_to_bitmap(*remote_data.bitmap_sent);
            }
        } else {
            // We don't have the memory to track the remote bitmap, we'll have to update it entirely
            remote_data.needs_tiles.for_each_intersected(send_tiles, [&](auto& tiles_to_send) {
                auto rect_to_send = tiles_to_send * RemoteData::tile_size;
                rect_to_send.intersect(rect());
                remote_gfx->connection().async_set_bitmap_data(remote_data.bitmap_id, { *this, rect_to_send });
                return AK::IterationDecision::Continue;
            });
        }
        remote_data.needs_tiles = remote_data.needs_tiles.shatter(send_tiles);
        return;
    } else {
        m_remote_data = nullptr;
    }
#endif
}

void Bitmap::invalidate_remote_rect(IntRect const& invalidate_rect)
{
    VERIFY(rect().contains(invalidate_rect));
#ifdef __serenity__
    if (!m_remote_data)
        return;
    auto& remote_data = *m_remote_data;
    VERIFY(remote_data.bitmap_id > 0);
    if (remote_data.session.ptr()) {
        IntRect invalidate_tiles { invalidate_rect.left() / RemoteData::tile_size, invalidate_rect.top() / RemoteData::tile_size, ceil_div(invalidate_rect.width(), RemoteData::tile_size), ceil_div(invalidate_rect.height(), RemoteData::tile_size) };
        if (!invalidate_tiles.is_empty())
            remote_data.needs_tiles.add(invalidate_tiles);
    } else {
        m_remote_data = nullptr;
    }
#endif
}

void Bitmap::remote_bitmap_sync([[maybe_unused]] u32 sync_tag)
{
#ifdef __serenity__
    if (!m_remote_data)
        return;
    auto& remote_data = *m_remote_data;
    VERIFY(remote_data.bitmap_id > 0);
    if (auto* remote_gfx = remote_data.session.ptr())
        remote_gfx->connection().async_sync_bitmap(remote_data.bitmap_id, sync_tag);
    else
        m_remote_data = nullptr;
#endif
}

void Bitmap::destroy_remote_data()
{
#ifdef __serenity__
    if (!m_remote_data)
        return;
    auto& remote_data = *m_remote_data;
    VERIFY(remote_data.bitmap_id > 0);
    if (auto* remote_gfx = remote_data.session.ptr())
        remote_gfx->connection().async_destroy_bitmap(remote_data.bitmap_id);
#endif
    m_remote_data = nullptr;
}

}
