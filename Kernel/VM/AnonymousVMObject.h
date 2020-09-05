/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
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

#include <Kernel/PhysicalAddress.h>
#include <Kernel/VM/VMObject.h>

namespace Kernel {

class AnonymousVMObject : public VMObject {
public:
    virtual ~AnonymousVMObject() override;

    static NonnullRefPtr<AnonymousVMObject> create_with_size(size_t);
    static RefPtr<AnonymousVMObject> create_for_physical_range(PhysicalAddress, size_t);
    static NonnullRefPtr<AnonymousVMObject> create_with_physical_page(PhysicalPage&);
    virtual RefPtr<VMObject> clone() override;

    virtual RefPtr<PhysicalPage> allocate_committed_page(size_t);

protected:
    explicit AnonymousVMObject(size_t, bool initialize_pages = true);
    explicit AnonymousVMObject(const AnonymousVMObject&);

    virtual const char* class_name() const override { return "AnonymousVMObject"; }

private:
    AnonymousVMObject(PhysicalAddress, size_t);

    AnonymousVMObject& operator=(const AnonymousVMObject&) = delete;
    AnonymousVMObject& operator=(AnonymousVMObject&&) = delete;
    AnonymousVMObject(AnonymousVMObject&&) = delete;

    virtual bool is_anonymous() const override { return true; }
};

}

AK_BEGIN_TYPE_TRAITS(Kernel::AnonymousVMObject)
static bool is_type(const Kernel::VMObject& vmobject) { return vmobject.is_anonymous(); }
AK_END_TYPE_TRAITS()
