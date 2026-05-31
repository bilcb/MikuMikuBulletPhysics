#pragma once
#include "core/vmd/types.h"
#include "core/model/model.h"

namespace mmp::anim {
    void evaluate(const vmd::VMDData& vmd, model::Model& model, float frame);
    void evaluateMorphs(const vmd::VMDData& vmd, model::Model& model, float frame);
    void evaluateIKKeys(const vmd::VMDData& vmd, model::Model& model, float frame);
}
