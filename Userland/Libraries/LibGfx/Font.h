/*
 * Copyright (c) 2020, Stephan Unverwerth <s.unverwerth@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/BitmapView.h>
#include <AK/MappedFile.h>
#include <AK/RefCounted.h>
#include <AK/RefPtr.h>
#include <AK/String.h>
#include <AK/Types.h>
#include <LibGfx/Bitmap.h>
#include <LibGfx/OneBitBitmap.h>
#include <LibGfx/Size.h>

namespace RemoteGfx {
class RemoteGfxSession;
}

namespace Gfx {

// FIXME: Make a MutableGlyphBitmap buddy class for FontEditor instead?
class GlyphBitmap : public OneBitBitmap {
public:
    GlyphBitmap() = default;
    GlyphBitmap(const unsigned* rows, IntSize size)
        : OneBitBitmap(OneBitBitmap::Type::GlyphBitmap, size)
        , m_rows(rows)
    {
    }
    GlyphBitmap(IntSize const&, BitmapView const&);
    ~GlyphBitmap();

    const unsigned* rows() const { return m_rows; }
    unsigned row(unsigned index) const { return m_rows[index]; }

    bool bit_at(int x, int y) const override { return row(y) & (1 << x); }
    void set_bit_at(int x, int y, bool b) override
    {
        auto& mutable_row = const_cast<unsigned*>(m_rows)[y];
        auto previous_bits = mutable_row;
        if (b)
            mutable_row |= 1 << x;
        else
            mutable_row &= ~(1 << x);
        if (previous_bits != mutable_row)
            set_dirty();
    }

    int width() const { return size().width(); }
    int height() const { return size().height(); }

private:
    const unsigned* m_rows { nullptr };
    bool m_own_rows { false };
};

class Glyph {
public:
    Glyph(GlyphBitmap&& glyph_bitmap, int left_bearing, int advance, int ascent)
        : m_glyph_bitmap(move(glyph_bitmap))
        , m_left_bearing(left_bearing)
        , m_advance(advance)
        , m_ascent(ascent)
    {
    }

    Glyph(RefPtr<Bitmap> bitmap, int left_bearing, int advance, int ascent)
        : m_bitmap(bitmap)
        , m_left_bearing(left_bearing)
        , m_advance(advance)
        , m_ascent(ascent)
    {
    }

    bool is_glyph_bitmap() const { return !m_bitmap; }
    GlyphBitmap glyph_bitmap() const { return m_glyph_bitmap; }
    RefPtr<Bitmap> bitmap() const { return m_bitmap; }
    int left_bearing() const { return m_left_bearing; }
    int advance() const { return m_advance; }
    int ascent() const { return m_ascent; }

private:
    GlyphBitmap m_glyph_bitmap;
    RefPtr<Bitmap> m_bitmap;
    int m_left_bearing;
    int m_advance;
    int m_ascent;
};

class Font : public RefCounted<Font> {
private:
    struct RemoteData {
        WeakPtr<RemoteGfx::RemoteGfxSession> session;
        int font_id { 0 };

        RemoteData(RemoteGfx::RemoteGfxSession&);
        ~RemoteData();
    };

public:
    enum class Type {
        Bitmap,
        Scaled
    };

    virtual NonnullRefPtr<Font> clone() const = 0;
    virtual ~Font() {};

    virtual Type font_type() const = 0;

    virtual u8 presentation_size() const = 0;

    virtual u16 weight() const = 0;
    virtual Glyph glyph(u32 code_point) const = 0;
    virtual bool contains_glyph(u32 code_point) const = 0;

    virtual u8 glyph_width(size_t ch) const = 0;
    virtual int glyph_or_emoji_width(u32 code_point) const = 0;
    virtual u8 glyph_height() const = 0;
    virtual int x_height() const = 0;

    virtual u8 min_glyph_width() const = 0;
    virtual u8 max_glyph_width() const = 0;
    virtual u8 glyph_fixed_width() const = 0;

    virtual u8 baseline() const = 0;
    virtual u8 mean_line() const = 0;

    virtual int width(const StringView&) const = 0;
    virtual int width(const Utf8View&) const = 0;
    virtual int width(const Utf32View&) const = 0;

    virtual String name() const = 0;

    virtual bool is_fixed_width() const = 0;

    virtual u8 glyph_spacing() const = 0;

    virtual size_t glyph_count() const = 0;

    virtual String family() const = 0;
    virtual String variant() const = 0;

    virtual String qualified_name() const = 0;

    Font const& bold_variant() const;

    virtual ReadonlyBytes bytes() const = 0;

    RemoteGfx::RemoteGfxSession* remote_session() { return m_remote_data ? m_remote_data->session.ptr() : nullptr; }
    int remote_font_id() const { return (m_remote_data && m_remote_data->session.ptr()) ? m_remote_data->font_id : 0; }
    int enable_remote_painting(bool);
    void send_to_remote();

private:
    mutable RefPtr<Gfx::Font> m_bold_variant;
    OwnPtr<RemoteData> m_remote_data;
};

}
