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

#include <AK/FlyString.h>
#include <AK/String.h>
#include <AK/StringBuilder.h>
#include <AK/Types.h>
#include <AK/Vector.h>

namespace Web {

class HTMLToken {
    friend class HTMLDocumentParser;
    friend class HTMLTokenizer;

public:
    enum class Type {
        Invalid,
        DOCTYPE,
        StartTag,
        EndTag,
        Comment,
        Character,
        EndOfFile,
    };

    static HTMLToken make_character(u32 codepoint)
    {
        HTMLToken token;
        token.m_type = Type::Character;
        token.m_comment_or_character.data.append(codepoint);
        return token;
    }

    bool is_doctype() const { return m_type == Type::DOCTYPE; }
    bool is_start_tag() const { return m_type == Type::StartTag; }
    bool is_end_tag() const { return m_type == Type::EndTag; }
    bool is_comment() const { return m_type == Type::Comment; }
    bool is_character() const { return m_type == Type::Character; }
    bool is_end_of_file() const { return m_type == Type::EndOfFile; }

    u32 codepoint() const
    {
        ASSERT(is_character());
        // FIXME: Handle non-ASCII codepoints properly.
        ASSERT(m_comment_or_character.data.length() == 1);
        return m_comment_or_character.data.string_view()[0];
    }

    bool is_parser_whitespace() const
    {
        // NOTE: The parser considers '\r' to be whitespace, while the tokenizer does not.
        if (!is_character())
            return false;
        switch (codepoint()) {
        case '\t':
        case '\n':
        case '\f':
        case '\r':
        case ' ':
            return true;
        default:
            return false;
        }
    }

    String tag_name() const
    {
        ASSERT(is_start_tag() || is_end_tag());
        return m_tag.tag_name.to_string();
    }

    bool is_self_closing() const
    {
        ASSERT(is_start_tag() || is_end_tag());
        return m_tag.self_closing;
    }

    bool has_acknowledged_self_closing_flag() const
    {
        ASSERT(is_self_closing());
        return m_tag.self_closing_acknowledged;
    }

    void acknowledge_self_closing_flag_if_set()
    {
        if (is_self_closing())
            m_tag.self_closing_acknowledged = true;
    }

    StringView attribute(const FlyString& attribute_name)
    {
        ASSERT(is_start_tag() || is_end_tag());
        for (auto& attribute : m_tag.attributes) {
            if (attribute_name == attribute.name_builder.string_view())
                return attribute.value_builder.string_view();
        }
        return {};
    }

    Type type() const { return m_type; }

    String to_string() const;

private:
    struct AttributeBuilder {
        StringBuilder name_builder;
        StringBuilder value_builder;
    };

    Type m_type { Type::Invalid };

    // Type::DOCTYPE
    struct {
        StringBuilder name;
        StringBuilder public_identifier;
        StringBuilder system_identifier;
        bool force_quirks { false };
    } m_doctype;

    // Type::StartTag
    // Type::EndTag
    struct {
        StringBuilder tag_name;
        bool self_closing { false };
        bool self_closing_acknowledged { false };
        Vector<AttributeBuilder> attributes;
    } m_tag;

    // Type::Comment
    // Type::Character
    struct {
        StringBuilder data;
    } m_comment_or_character;
};

}
