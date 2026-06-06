#pragma once
#include <btBulletDynamicsCommon.h>
#include <vector>

namespace mmbp::model {
class Model;
}
namespace mmbp::physics {
class World;
}
namespace mmbp::vmd {
struct VMDData;
}
namespace mmbp::anim {
struct AnimationState;
}

namespace mmbp::bridge {

class PhysicsPipeline {
public:
  struct StepResult {
    int boneCount;
  };

  StepResult step(anim::AnimationState &animState, model::Model &model,
                  physics::World *world, const vmd::VMDData &vmdData,
                  float frame, float delta,
                  const std::vector<btMatrix3x3> &restMatrices, float scale,
                  float *outLocs, float *outQuats, int boneCount);

  void updatePostPhysicsFlag(const model::Model &model);

  StepResult resetToAnimation(anim::AnimationState &animState,
                              model::Model &model, physics::World *world,
                              const vmd::VMDData &vmdData, float frame,
                              const std::vector<btMatrix3x3> &restMatrices,
                              float scale, float *outLocs, float *outQuats,
                              int boneCount);

  StepResult evalOnly(anim::AnimationState &animState, model::Model &model,
                      const vmd::VMDData &vmdData, float frame,
                      const std::vector<btMatrix3x3> &restMatrices, float scale,
                      float *outLocs, float *outQuats, int boneCount);

private:
  void animate(anim::AnimationState &animState, model::Model &model,
               const vmd::VMDData &vmdData, float frame);
  void prePhysics(model::Model &model);
  void simulate(physics::World *world, model::Model &model, float delta);
  void postPhysics(model::Model &model);
  void output(model::Model &model, const std::vector<btMatrix3x3> &restMatrices,
              float scale, float *outLocs, float *outQuats, int boneCount);

  bool m_hasPostPhysicsBones = false;
};

} // namespace mmbp::bridge
