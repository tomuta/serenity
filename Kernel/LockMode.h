/*
 * Copyright (c) 2020, the SerenityOS developers.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

namespace Kernel {

enum class LockMode : u8 {
    Unlocked,
    Shared,
    Exclusive
};

}
