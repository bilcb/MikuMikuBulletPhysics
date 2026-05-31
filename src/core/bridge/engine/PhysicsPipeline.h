#pragma once
#include <vector>
#include <btBulletDynamicsCommon.h>

namespace mmp::model { class Model; }
namespace mmp::physics { class World; }
namespace mmp::vmd { struct VMDData; }

namespace mmp::bridge {

class PhysicsPipeline {
public:
    struct StepResult {
        int boneCount;
    };

    StepResult step(
        model::Model& model,
        physics::World* world,
        const vmd::VMDData& vmdData,
        float frame,
        float delta,
        const std::vector<btMatrix3x3>& restMatrices,
        const std::vector<btVector3>& restPosOffsets,
        float scale,
        float* outLocs,
        float* outQuats,
        int boneCount
    );

private:
    void animate(model::Model& model, const vmd::VMDData& vmdData, float frame);
    void convertVmdToBlenderSpace(model::Model& model);
    void prePhysics(model::Model& model);
    void simulate(physics::World* world, model::Model& model, float delta);
    void postPhysics(model::Model& model);
    void output(
        model::Model& model,
        const std::vector<btMatrix3x3>& restMatrices,
        const std::vector<btVector3>& restPosOffsets,
        float scale,
        float* outLocs,
        float* outQuats,
        int boneCount
    );
};

} // namespace mmp::bridge
