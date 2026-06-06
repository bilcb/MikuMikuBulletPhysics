#include "parser.h"
#include "core/math/converter.h"
#include "core/util/encoding.h"
#include "core/util/logger.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

bool clampFinite(float &v, const char *field, const char *ctx) {
  if (!std::isfinite(v)) {
    g_logger.warn("[MMBP] PMX %s: %s is non-finite (NaN/Inf), clamping to 0",
                  ctx, field);
    v = 0.0f;
    return true;
  }
  return false;
}
bool clampFiniteVec3(btVector3 &v, const char *field, const char *ctx) {
  bool fixed = false;
  if (!std::isfinite(v.x())) {
    v.setX(0);
    fixed = true;
  }
  if (!std::isfinite(v.y())) {
    v.setY(0);
    fixed = true;
  }
  if (!std::isfinite(v.z())) {
    v.setZ(0);
    fixed = true;
  }
  if (fixed)
    g_logger.warn(
        "[MMBP] PMX %s: %s contains non-finite component(s), clamping to 0",
        ctx, field);
  return fixed;
}
bool clampFiniteQuat(btQuaternion &q, const char *field, const char *ctx) {
  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      !std::isfinite(q.w())) {
    g_logger.warn(
        "[MMBP] PMX %s: %s is non-finite, resetting to identity quaternion",
        ctx, field);
    q = btQuaternion(0, 0, 0, 1);
    return true;
  }
  return false;
}
bool clampNonNeg(float &v, const char *field, const char *ctx) {
  if (v < 0.0f) {
    g_logger.warn("[MMBP] PMX %s: %s = %g (negative), clamping to 0", ctx,
                  field, v);
    v = 0.0f;
    return true;
  }
  return false;
}
bool clampNonNegVec3(btVector3 &v, const char *field, const char *ctx) {
  bool fixed = false;
  if (v.x() < 0) {
    g_logger.warn("[MMBP] PMX %s: %s.x = %g (negative)", ctx, field, v.x());
    v.setX(0);
    fixed = true;
  }
  if (v.y() < 0) {
    g_logger.warn("[MMBP] PMX %s: %s.y = %g (negative)", ctx, field, v.y());
    v.setY(0);
    fixed = true;
  }
  if (v.z() < 0) {
    g_logger.warn("[MMBP] PMX %s: %s.z = %g (negative)", ctx, field, v.z());
    v.setZ(0);
    fixed = true;
  }
  return fixed;
}
bool checkRange(int v, int lo, int hi, const char *field, const char *ctx) {
  if (v < lo || v > hi) {
    g_logger.warn("[MMBP] PMX %s: %s = %d (expected [%d,%d])", ctx, field, v,
                  lo, hi);
    return true;
  }
  return false;
}
} // namespace

struct BinReader {
  const uint8_t *data;
  int size;
  int pos;
  bool hadError;

  BinReader(const uint8_t *d, int s)
      : data(d), size(s), pos(0), hadError(false) {}

  bool eof() const { return pos >= size; }
  bool bad() const { return pos > size || hadError; }

  uint8_t readU8() {
    if (pos >= size) {
      hadError = true;
      return 0;
    }
    return data[pos++];
  }
  uint16_t readU16() {
    if (pos + 2 > size) {
      pos = size;
      hadError = true;
      return 0;
    }
    uint16_t v;
    std::memcpy(&v, data + pos, 2);
    pos += 2;
    return v;
  }
  uint32_t readU32() {
    if (pos + 4 > size) {
      pos = size;
      hadError = true;
      return 0;
    }
    uint32_t v;
    std::memcpy(&v, data + pos, 4);
    pos += 4;
    return v;
  }
  int32_t readI32() { return static_cast<int32_t>(readU32()); }
  float readF32() {
    if (pos + 4 > size) {
      pos = size;
      hadError = true;
      return 0;
    }
    float v;
    std::memcpy(&v, data + pos, 4);
    pos += 4;
    return v;
  }

  int32_t readIdx(int sz) {
    switch (sz) {
    case 1: {
      int8_t v = static_cast<int8_t>(readU8());
      if (v == -1)
        return -1;
      if (v < 0) {
        hadError = true;
        return -1;
      }
      return static_cast<int32_t>(v);
    }
    case 2: {
      int16_t v = static_cast<int16_t>(readU16());
      if (v == -1)
        return -1;
      if (v < 0) {
        hadError = true;
        return -1;
      }
      return static_cast<int32_t>(v);
    }
    case 4: {
      int32_t v = readI32();
      if (v < 0 && v != -1) {
        hadError = true;
        return -1;
      }
      return v;
    }
    default:
      return -1;
    }
  }
  int32_t readIdxUnsigned(int sz) {
    switch (sz) {
    case 1:
      return readU8();
    case 2:
      return readU16();
    case 4: {
      uint32_t v = readU32();
      return v > 0x7FFFFFFFu ? -1 : static_cast<int32_t>(v);
    }
    default:
      return -1;
    }
  }

  std::string readString(int encoding) {
    int32_t length = readI32();
    if (length <= 0)
      return {};
    if (length > size - pos) {
      pos = size;
      return {};
    }
    std::string result =
        mmbp::util::decodePmxString(data + pos, length, encoding);
    pos += length;
    return result;
  }

  btVector3 readVec3() {
    float x = readF32();
    float y = readF32();
    float z = readF32();
    return btVector3(x, y, z);
  }

  btQuaternion readQuat() {
    float x = readF32();
    float y = readF32();
    float z = readF32();
    float w = readF32();
    return btQuaternion(x, y, z, w);
  }

  void readFloats(float *out, int count) {
    for (int i = 0; i < count; i++)
      out[i] = readF32();
  }

  void skipVec3() { skip(12); }
  void skip(int n) {
    if (n < 0)
      n = 0;
    pos += n;
    if (pos > size)
      pos = size;
  }
};

namespace mmbp::pmx {

static bool parseBones(BinReader &reader, PMXData &out, ParseError &err,
                       int idxBone, int encoding) {
  int32_t count = reader.readI32();
  if (count < 0) {
    err.message = "negative bone count";
    return false;
  }
  if (count > limits::MAX_BONES) {
    err.message = "bone count exceeds limit";
    return false;
  }
  out.bones.resize(count);
  for (int32_t i = 0; i < count; i++) {
    auto &b = out.bones[i];
    b.name = reader.readString(encoding);
    b.nameEn = reader.readString(encoding);
    b.position = reader.readVec3();
    b.parentIdx = reader.readIdx(idxBone);
    b.deformDepth = reader.readI32();
    b.flags = reader.readU16();

    b.isRotatable = (b.flags & 0x0002) != 0;
    b.isMovable = (b.flags & 0x0004) != 0;
    b.visible = (b.flags & 0x0008) != 0;
    b.isControllable = (b.flags & 0x0010) != 0;
    b.isIK = (b.flags & 0x0020) != 0;
    b.hasAdditionalRotate = (b.flags & 0x0100) != 0;
    b.hasAdditionalLocate = (b.flags & 0x0200) != 0;
    b.appendLocal = (b.flags & 0x0080) != 0;
    b.hasFixedAxis = (b.flags & 0x0400) != 0;
    b.hasLocalAxes = (b.flags & 0x0800) != 0;
    b.transAfterPhys = (b.flags & 0x1000) != 0;

    if (b.flags & 0x0001) {
      b.displayConnection = reader.readIdx(idxBone);
    } else {
      b.displayOffset = reader.readVec3();
      b.displayConnection = -1;
    }

    if (b.hasAdditionalRotate || b.hasAdditionalLocate) {
      b.appendBoneIdx = reader.readIdx(idxBone);
      b.appendWeight = reader.readF32();

      if (b.appendWeight < 0.0f)
        b.appendWeight = 0.0f;
      if (b.appendWeight > 1.0f)
        b.appendWeight = 1.0f;
    }

    if (b.hasFixedAxis) {
      b.fixedAxis = reader.readVec3();
    }

    if (b.hasLocalAxes) {
      b.localAxisX = reader.readVec3();
      b.localAxisZ = reader.readVec3();
    }

    if (b.flags & 0x2000) {
      b.externalTransKey = reader.readI32();
    }

    if (b.isIK) {
      b.ikTargetIdx = reader.readIdx(idxBone);
      b.ikLoopCount = reader.readI32();
      b.ikRotationLimit = reader.readF32();
      int32_t linkCount = reader.readI32();
      if (linkCount < 0) {
        err.message = "negative IK link count";
        return false;
      }
      if (linkCount > limits::MAX_IK_LINKS) {
        err.message = "IK link count exceeds limit";
        return false;
      }
      b.ikLinks.resize(linkCount);
      for (int32_t j = 0; j < linkCount; j++) {
        auto &link = b.ikLinks[j];
        link.boneIdx = reader.readIdx(idxBone);
        uint8_t hasLimit = reader.readU8();
        link.hasLimit = (hasLimit == 1);
        if (link.hasLimit) {
          link.limitMin = reader.readVec3();
          link.limitMax = reader.readVec3();
        }
      }
    }

    std::string ctx = "bone[" + std::to_string(i) + "]";
    clampFiniteVec3(b.position, "position", ctx.c_str());
    if (b.isIK) {
      checkRange(b.ikLoopCount, 0, 10000, "ikLoopCount", ctx.c_str());
      if (b.ikLoopCount > 10000)
        b.ikLoopCount = 10000;
      clampFinite(b.ikRotationLimit, "ikRotationLimit", ctx.c_str());
      if (b.ikRotationLimit < 0)
        b.ikRotationLimit = -b.ikRotationLimit;
      for (size_t j = 0; j < b.ikLinks.size(); j++) {
        clampFiniteVec3(b.ikLinks[j].limitMin, "ikLink.limitMin", ctx.c_str());
        clampFiniteVec3(b.ikLinks[j].limitMax, "ikLink.limitMax", ctx.c_str());
      }
    }
    if (b.hasAdditionalRotate || b.hasAdditionalLocate) {
      clampFinite(b.appendWeight, "appendWeight", ctx.c_str());
    }
  }
  return true;
}

static bool parseMorphs(BinReader &reader, PMXData &out, ParseError &err,
                        int idxBone, int idxMorph, int idxVertex,
                        int idxMaterial, int idxRigid, int encoding) {
  int32_t count = reader.readI32();
  if (count < 0) {
    err.message = "negative morph count";
    return false;
  }
  if (count > limits::MAX_MORPHS) {
    err.message = "morph count exceeds limit";
    return false;
  }
  out.morphs.resize(count);
  for (int32_t i = 0; i < count; i++) {
    auto &m = out.morphs[i];
    m.name = reader.readString(encoding);
    m.nameEn = reader.readString(encoding);
    m.category = reader.readU8();
    uint8_t morphType = reader.readU8();
    int32_t dataCount = reader.readI32();
    if (dataCount < 0)
      dataCount = 0;

    switch (morphType) {
    case 0:
      m.groupOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        m.groupOffsets[j].idx = reader.readIdx(idxMorph);
        m.groupOffsets[j].factor = reader.readF32();
      }
      break;
    case 1:
      m.vertexOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        m.vertexOffsets[j].idx = reader.readIdxUnsigned(idxVertex);
        m.vertexOffsets[j].offset = reader.readVec3();
      }
      break;
    case 2:
      m.boneOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        m.boneOffsets[j].idx = reader.readIdx(idxBone);
        m.boneOffsets[j].translation = reader.readVec3();
        m.boneOffsets[j].rotation = reader.readQuat();

        if (m.boneOffsets[j].rotation.x() == 0.0f &&
            m.boneOffsets[j].rotation.y() == 0.0f &&
            m.boneOffsets[j].rotation.z() == 0.0f &&
            m.boneOffsets[j].rotation.w() == 0.0f) {
          m.boneOffsets[j].rotation = btQuaternion(0, 0, 0, 1);
        }
      }
      break;
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      m.uvOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        m.uvOffsets[j].idx = reader.readIdxUnsigned(idxVertex);
        reader.readFloats(m.uvOffsets[j].offset, 4);
      }
      break;
    case 8:
      m.materialOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        auto &mm = m.materialOffsets[j];
        mm.idx = reader.readIdx(idxMaterial);
        mm.op = reader.readU8();
        reader.readFloats(mm.diffuse, 4);
        reader.readFloats(mm.specular, 3);
        mm.shininess = reader.readF32();
        reader.readFloats(mm.ambient, 3);
        reader.readFloats(mm.edgeColor, 4);
        mm.edgeSize = reader.readF32();
        reader.readFloats(mm.texture, 4);
        reader.readFloats(mm.sphere, 4);
        reader.readFloats(mm.toon, 4);
      }
      break;
    case 9:
      m.flipOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        m.flipOffsets[j].idx = reader.readIdx(idxMorph);
        m.flipOffsets[j].factor = reader.readF32();
      }
      break;
    case 10:
      m.impulseOffsets.resize(dataCount);
      for (int32_t j = 0; j < dataCount; j++) {
        auto &io = m.impulseOffsets[j];
        io.idx = reader.readIdx(idxRigid);
        io.local = (reader.readU8() != 0);
        io.velocity = reader.readVec3();
        io.torque = reader.readVec3();
      }
      break;
    }
    if (reader.bad()) {
      err.message = "Morph read failed";
      return false;
    }
  }
  return true;
}

static bool parseDisplayFrames(BinReader &reader, PMXData &out, ParseError &err,
                               int idxBone, int idxMorph, int encoding) {
  int32_t frameCount = reader.readI32();
  if (frameCount < 0) {
    err.message = "negative display frame count";
    return false;
  }
  out.displayFrames.resize(frameCount);
  for (int32_t i = 0; i < frameCount; i++) {
    auto &df = out.displayFrames[i];
    df.name = reader.readString(encoding);
    reader.readString(encoding);
    df.isSpecial = (reader.readU8() != 0);
    int32_t targetCount = reader.readI32();
    df.targets.reserve(targetCount);
    for (int32_t j = 0; j < targetCount; j++) {
      uint8_t type = reader.readU8();
      int32_t idx =
          (type == 0) ? reader.readIdx(idxBone) : reader.readIdx(idxMorph);
      df.targets.push_back({type, idx});
    }
  }
  return true;
}

static bool parseRigidBodies(BinReader &reader, PMXData &out, ParseError &err,
                             int idxBone, int encoding) {
  int32_t count = reader.readI32();
  if (count < 0) {
    err.message = "negative rigid body count";
    return false;
  }
  if (count > limits::MAX_RIGID_BODIES) {
    err.message = "rigid body count exceeds limit";
    return false;
  }
  out.rigidBodies.resize(count);
  for (int32_t i = 0; i < count; i++) {
    auto &rb = out.rigidBodies[i];
    rb.name = reader.readString(encoding);
    rb.nameEn = reader.readString(encoding);
    rb.boneIdx = reader.readIdx(idxBone);
    rb.group = reader.readU8();
    rb.groupMask = reader.readU16();
    rb.shapeType = reader.readU8();
    rb.size = reader.readVec3();
    rb.position = reader.readVec3();

    float eulerX = reader.readF32(), eulerY = reader.readF32(),
          eulerZ = reader.readF32();
    rb.rotation = mmbp::math::eulerToQuaternionYxz(eulerY, eulerX, eulerZ);
    rb.eulerRotation = btVector3(eulerX, eulerY, eulerZ);
    rb.mass = reader.readF32();
    rb.linearDamping = reader.readF32();
    rb.angularDamping = reader.readF32();
    rb.restitution = reader.readF32();
    rb.friction = reader.readF32();
    rb.mode = reader.readU8();

    std::string ctx = "rigid body[" + std::to_string(i) + "]";
    checkRange(rb.shapeType, 0, 2, "shapeType", ctx.c_str());
    checkRange(rb.mode, 0, 2, "mode", ctx.c_str());
    checkRange(rb.group, 0, 15, "group", ctx.c_str());
    clampFiniteVec3(rb.position, "position", ctx.c_str());
    clampNonNegVec3(rb.size, "size", ctx.c_str());
    clampFiniteVec3(rb.size, "size", ctx.c_str());
    clampFiniteQuat(rb.rotation, "rotation", ctx.c_str());
    clampFinite(rb.mass, "mass", ctx.c_str());
    clampNonNeg(rb.mass, "mass", ctx.c_str());
    clampFinite(rb.linearDamping, "linearDamping", ctx.c_str());
    clampNonNeg(rb.linearDamping, "linearDamping", ctx.c_str());
    clampFinite(rb.angularDamping, "angularDamping", ctx.c_str());
    clampNonNeg(rb.angularDamping, "angularDamping", ctx.c_str());
    clampFinite(rb.restitution, "restitution", ctx.c_str());
    clampNonNeg(rb.restitution, "restitution", ctx.c_str());
    clampFinite(rb.friction, "friction", ctx.c_str());
    clampNonNeg(rb.friction, "friction", ctx.c_str());
  }
  return true;
}

static bool parseJoints(BinReader &reader, PMXData &out, ParseError &err,
                        int idxRigid, int encoding) {
  int32_t count = reader.readI32();
  if (count < 0) {
    err.message = "negative joint count";
    return false;
  }
  if (count > limits::MAX_JOINTS) {
    err.message = "joint count exceeds limit";
    return false;
  }
  out.joints.resize(count);
  for (int32_t i = 0; i < count; i++) {
    auto &j = out.joints[i];
    j.name = reader.readString(encoding);
    j.nameEn = reader.readString(encoding);
    j.mode = reader.readU8();
    j.rigidBodyAIdx = reader.readIdx(idxRigid);
    j.rigidBodyBIdx = reader.readIdx(idxRigid);
    j.position = reader.readVec3();
    float eulerX = reader.readF32(), eulerY = reader.readF32(),
          eulerZ = reader.readF32();
    j.rotation = mmbp::math::eulerToQuaternionYxz(eulerY, eulerX, eulerZ);
    j.linearLowerLimit = reader.readVec3();
    j.linearUpperLimit = reader.readVec3();
    j.angularLowerLimit = reader.readVec3();
    j.angularUpperLimit = reader.readVec3();
    j.springTranslate = reader.readVec3();
    j.springRotate = reader.readVec3();

    std::string ctx = "joint[" + std::to_string(i) + "]";
    clampFiniteVec3(j.position, "position", ctx.c_str());
    clampFiniteQuat(j.rotation, "rotation", ctx.c_str());
    clampFiniteVec3(j.linearLowerLimit, "linearLowerLimit", ctx.c_str());
    clampFiniteVec3(j.linearUpperLimit, "linearUpperLimit", ctx.c_str());
    clampFiniteVec3(j.angularLowerLimit, "angularLowerLimit", ctx.c_str());
    clampFiniteVec3(j.angularUpperLimit, "angularUpperLimit", ctx.c_str());
    clampFiniteVec3(j.springTranslate, "springTranslate", ctx.c_str());
    clampFiniteVec3(j.springRotate, "springRotate", ctx.c_str());
  }
  return true;
}

static bool parse_impl(PMXData &out, ParseError &err, const uint8_t *data,
                       int size) {
  if (!data || size < 32) {
    err.message = "Invalid data: null pointer or size < 32 bytes";
    return false;
  }
  BinReader reader(data, size);

  if (reader.pos + 4 > reader.size ||
      std::memcmp(reader.data + reader.pos, "PMX ", 4) != 0) {
    err.message = "Invalid magic bytes: expected 'PMX ' header";
    return false;
  }
  reader.skip(4);

  float version = reader.readF32();
  if (reader.bad() || version < 2.0f || version > 2.1f) {
    err.message = "Unsupported PMX version: expected 2.0 or 2.1";
    return false;
  }

  uint8_t dataSize = reader.readU8();
  if (dataSize != 8) {

    g_logger.warn(
        "[MMBP] PMX header dataSize=%d, expected 8 — attempting to continue",
        dataSize);
  }
  int encoding = reader.readU8();
  uint8_t addUV = reader.readU8();

  uint8_t idxVertex = reader.readU8();
  uint8_t idxTexture = reader.readU8();
  uint8_t idxMaterial = reader.readU8();
  uint8_t idxBone = reader.readU8();
  uint8_t idxMorph = reader.readU8();
  uint8_t idxRigid = reader.readU8();

  out.name = reader.readString(encoding);
  reader.readString(encoding);
  out.comment = reader.readString(encoding);
  reader.readString(encoding);

  {
    int32_t count = reader.readI32();
    for (int32_t i = 0; i < count; i++) {
      reader.readVec3();
      reader.readVec3();
      reader.skip(8);
      for (int j = 0; j < addUV; j++)
        reader.skip(16);
      uint8_t weightType = reader.readU8();
      auto readBoneWeight = [&](int n) {
        for (int k = 0; k < n; k++)
          reader.readIdx(idxBone);
        for (int k = 0; k < n; k++)
          reader.readF32();
      };
      switch (weightType) {
      case 0:
        reader.readIdx(idxBone);
        break;
      case 1:
        reader.readIdx(idxBone);
        reader.readIdx(idxBone);
        reader.readF32();
        break;
      case 2:
        readBoneWeight(4);
        break;
      case 3:
        reader.readIdx(idxBone);
        reader.readIdx(idxBone);
        reader.readF32();
        reader.readVec3();
        reader.readVec3();
        reader.readVec3();
        break;
      case 4:
        readBoneWeight(4);
        break;
      default:
        err.message = "Unknown vertex weight type";
        return false;
      }
      reader.readF32();
    }
  }

  {
    int32_t faceCount = reader.readI32();
    if (faceCount <= 0)
      faceCount = 0;
    int64_t skipBytes = static_cast<int64_t>(faceCount) * idxVertex;
    if (skipBytes > reader.size - reader.pos) {
      reader.pos = reader.size;
    } else {
      reader.pos += static_cast<int>(skipBytes);
    }
  }

  {
    int32_t count = reader.readI32();
    for (int32_t i = 0; i < count; i++)
      reader.readString(encoding);
  }

  {
    int32_t count = reader.readI32();
    for (int32_t i = 0; i < count; i++) {
      reader.readString(encoding);
      reader.readString(encoding);
      reader.skip(16 + 12 + 4 + 12 + 1);
      reader.skip(16 + 4);
      reader.readIdx(idxTexture);
      reader.readIdx(idxTexture);
      reader.readU8();
      uint8_t toonMode = reader.readU8();
      if (toonMode == 0)
        reader.readIdx(idxTexture);
      else
        reader.readU8();
      reader.readString(encoding);
      reader.readI32();
    }
  }

  if (!parseBones(reader, out, err, idxBone, encoding))
    return false;
  if (!parseMorphs(reader, out, err, idxBone, idxMorph, idxVertex, idxMaterial,
                   idxRigid, encoding))
    return false;

  {
    std::vector<int> visited(out.morphs.size(), 0);
    for (size_t i = 0; i < out.morphs.size(); i++) {
      const auto &m = out.morphs[i];
      if (m.groupOffsets.empty())
        continue;
      std::vector<size_t> path;

      std::function<bool(size_t)> dfs = [&](size_t idx) -> bool {
        if (idx >= out.morphs.size())
          return false;
        if (visited[idx] == 2)
          return false;
        if (visited[idx] == 1) {
          g_logger.warn("[MMBP] Group Morph cycle detected at morph '%s' "
                        "(idx=%zu) — clearing its factor to skip",
                        out.morphs[idx].name.c_str(), idx);

          out.morphs[idx].groupOffsets.clear();
          return true;
        }
        visited[idx] = 1;
        for (const auto &go : out.morphs[idx].groupOffsets) {
          if (go.idx >= 0 && static_cast<size_t>(go.idx) < out.morphs.size()) {
            if (dfs(static_cast<size_t>(go.idx)))
              break;
          }
        }
        visited[idx] = 2;
        return false;
      };
      if (visited[i] == 0)
        dfs(i);
    }
  }

  if (!parseDisplayFrames(reader, out, err, idxBone, idxMorph, encoding))
    return false;
  if (!parseRigidBodies(reader, out, err, idxBone, encoding))
    return false;
  if (!parseJoints(reader, out, err, idxRigid, encoding))
    return false;

  for (size_t i = 0; i < out.joints.size(); i++) {
    const auto &j = out.joints[i];
    if (j.rigidBodyAIdx < 0 ||
        (size_t)j.rigidBodyAIdx >= out.rigidBodies.size())
      g_logger.warn(
          "[MMBP] PMX joint[%zu] rigidBodyAIdx=%d out of range [0,%zu)", i,
          j.rigidBodyAIdx, out.rigidBodies.size());
    if (j.rigidBodyBIdx < 0 ||
        (size_t)j.rigidBodyBIdx >= out.rigidBodies.size())
      g_logger.warn(
          "[MMBP] PMX joint[%zu] rigidBodyBIdx=%d out of range [0,%zu)", i,
          j.rigidBodyBIdx, out.rigidBodies.size());
  }

  if (version >= 2.1f && reader.pos < reader.size) {
    int32_t sbCount = reader.readI32();
    if (!reader.bad() && sbCount > 0) {
      for (int32_t i = 0; i < sbCount; i++) {
        if (reader.bad())
          break;
        reader.readString(encoding);
        reader.readString(encoding);
        reader.readIdx(idxRigid);
        reader.readIdx(idxBone);
        reader.skip(65);
        int32_t anchorCount = reader.readI32();
        for (int32_t a = 0; a < anchorCount; a++) {
          reader.readIdx(idxRigid);
          reader.readIdx(idxRigid);
        }
        int32_t pinCount = reader.readI32();
        for (int32_t p = 0; p < pinCount; p++) {
          reader.readIdx(idxRigid);
        }
      }
    }
  }

  if (reader.pos < reader.size) {
    reader.pos = reader.size;
  }

  if (reader.bad()) {
    err.message = "Unexpected end of file";
    return false;
  }
  return true;
}

ParseResult<PMXData> parse_memory(const uint8_t *data, int size) {
  ParseResult<PMXData> result;
  if (!parse_impl(result.m_value, result.m_error, data, size)) {
  }
  return result;
}

ParseResult<PMXData> parse_file(const char *path) {
  ParseResult<PMXData> result;
#ifdef _WIN32
  int pathLen = MultiByteToWideChar(CP_UTF8, 0, path, -1, nullptr, 0);
  FILE *fp = nullptr;
  if (pathLen > 0) {
    std::wstring wpath(pathLen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path, -1, &wpath[0], pathLen);
    fp = _wfopen(wpath.c_str(), L"rb");
  }
#else
  FILE *fp = fopen(path, "rb");
#endif
  if (!fp) {
    result.m_error.message = "Cannot open PMX file";
    return result;
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz <= 0) {
    fclose(fp);
    result.m_error.message = "PMX file is empty or unreadable";
    return result;
  }
  std::vector<uint8_t> buf(sz);
  if (fread(buf.data(), 1, sz, fp) != static_cast<size_t>(sz)) {
    fclose(fp);
    result.m_error.message = "Failed to read PMX file";
    return result;
  }
  fclose(fp);
  return parse_memory(buf.data(), static_cast<int>(sz));
}

} // namespace mmbp::pmx
