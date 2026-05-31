#pragma once
#include "core/model/model.h"

namespace mmp::ik {
    void solve(model::Model& model, const model::BoneNode& ikBone);
}
