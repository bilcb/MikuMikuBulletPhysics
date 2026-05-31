#include "parser.h"
#include "core/math/converter.h"
#include "core/util/encoding.h"
#include <cstring>
#include <cstdio>
#include <vector>

struct BinReader {
    const uint8_t* data;
    int size;
    int pos;

    BinReader(const uint8_t* d, int s)
        : data(d), size(s), pos(0) {}

    bool eof() const { return pos >= size; }
    bool bad() const { return pos > size; }

    uint8_t readU8()  { if (pos >= size) return 0; return data[pos++]; }
    uint16_t readU16() { if (pos + 2 > size) { pos = size; return 0; } uint16_t v; std::memcpy(&v, data + pos, 2); pos += 2; return v; }
    uint32_t readU32() { if (pos + 4 > size) { pos = size; return 0; } uint32_t v; std::memcpy(&v, data + pos, 4); pos += 4; return v; }
    int32_t readI32()  { return static_cast<int32_t>(readU32()); }
    float readF32()    { if (pos + 4 > size) { pos = size; return 0; } float v; std::memcpy(&v, data + pos, 4); pos += 4; return v; }

    int32_t readIdx(int sz) {
        switch (sz) {
        case 1: { int8_t v = static_cast<int8_t>(readU8()); return (v != -1) ? static_cast<int32_t>(v) : -1; }
        case 2: { int16_t v = static_cast<int16_t>(readU16()); return (v != -1) ? static_cast<int32_t>(v) : -1; }
        case 4: return readI32();
        default: return -1;
        }
    }
    int32_t readIdxUnsigned(int sz) {
        switch (sz) {
        case 1: return readU8();
        case 2: return readU16();
        case 4: return static_cast<int32_t>(readU32());
        default: return -1;
        }
    }

    std::string readString(int encoding) {
        int32_t length = readI32();
        if (length <= 0) return {};
        if (pos + length > size) { pos = size; return {}; }
        std::string result = mmp::util::decodePmxString(data + pos, length, encoding);
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

    void readFloats(float* out, int count) {
        for (int i = 0; i < count; i++) out[i] = readF32();
    }

    void skip(int n) { pos += n; if (pos > size) pos = size; }
};

namespace mmp::pmx {

static bool parse_impl(PMXData& out, ParseError& err, const uint8_t* data, int size) {
    if (!data || size < 32) {
        err.message = "Invalid data";
        return false;
    }
    BinReader reader(data, size);

    // magic
    if (reader.pos + 4 > reader.size || std::memcmp(reader.data + reader.pos, "PMX ", 4) != 0) {
        err.message = "Invalid magic bytes";return false;
    }
    reader.skip(4);

    // version
    float version = reader.readF32();
    if (reader.bad() || version < 2.0f || version > 2.1f) {
        err.message = "Unsupported PMX version";return false;
    }

    // header: data size, encoding, additional UV count, index sizes
    uint8_t dataSize = reader.readU8();
    if (dataSize != 8) { err.message = "Invalid PMX header dataSize";return false; }
    int encoding = reader.readU8();
    uint8_t addUV = reader.readU8();
    (void)addUV;
    uint8_t idxVertex    = reader.readU8();
    uint8_t idxTexture   = reader.readU8();
    uint8_t idxMaterial  = reader.readU8();
    uint8_t idxBone      = reader.readU8();
    uint8_t idxMorph     = reader.readU8();
    uint8_t idxRigid     = reader.readU8();

    // model name, nameEn, comment, commentEn
    out.name = reader.readString(encoding);
    reader.readString(encoding);  // skip nameEn
    out.comment = reader.readString(encoding);
    reader.readString(encoding);  // skip commentEn

    // skip vertices
    {
        int32_t count = reader.readI32();
        for (int32_t i = 0; i < count; i++) {
            reader.readVec3(); reader.readVec3();  // pos, normal
            reader.skip(8);                    // uv
            for (int j = 0; j < addUV; j++) reader.skip(16);
            uint8_t weightType = reader.readU8();
            auto readBoneWeight = [&](int n) {
                for (int k = 0; k < n; k++) reader.readIdx(idxBone);
                for (int k = 0; k < n; k++) reader.readF32();
            };
            switch (weightType) {
            case 0: reader.readIdx(idxBone); break;               // BDEF1: 1 index + 0 weights
            case 1: reader.readIdx(idxBone); reader.readIdx(idxBone); reader.readF32(); break; // BDEF2: 2 indices + 1 weight
            case 2: readBoneWeight(4); break;                      // BDEF4: 4 indices + 4 weights
            case 3: reader.readIdx(idxBone); reader.readIdx(idxBone); reader.readF32(); reader.readVec3(); reader.readVec3(); reader.readVec3(); break; // SDEF
            case 4: readBoneWeight(4); break;                      // QDEF: 4 indices + 4 weights
            default:
                err.message = "Unknown vertex weight type";
                return false;
            }
            reader.readF32();  // edge scale
        }
    }

    // skip faces
    {
        int32_t faceCount = reader.readI32();
        int32_t tris = faceCount / 3;
        reader.skip(tris * 3 * idxVertex);
    }

    // skip textures
    {
        int32_t count = reader.readI32();
        for (int32_t i = 0; i < count; i++) reader.readString(encoding);
    }

    // skip materials
    {
        int32_t count = reader.readI32();
        for (int32_t i = 0; i < count; i++) {
            reader.readString(encoding); reader.readString(encoding);   // name, nameEn
            reader.skip(16 + 12 + 4 + 12 + 1);                      // diffuse, specular, power, ambient, drawMode
            reader.skip(16 + 4);                                     // edge color, edge size
            reader.readIdx(idxTexture);                              // texture index
            reader.readIdx(idxTexture);                              // sphere texture index
            reader.readU8();                                         // sphere mode
            uint8_t toonMode = reader.readU8();
            if (toonMode == 0) reader.readIdx(idxTexture);
            else reader.readU8();
            reader.readString(encoding);                            // memo
            reader.readI32();                                       // face vertex count
        }
    }

    // bones
    {
        int32_t count = reader.readI32();
        out.bones.resize(count);
        for (int32_t i = 0; i < count; i++) {
            auto& b = out.bones[i];
            b.name = reader.readString(encoding);
            b.nameEn = reader.readString(encoding);
            b.position = reader.readVec3();
            b.parentIdx = reader.readIdx(idxBone);
            b.deformDepth = reader.readI32();
            b.flags = reader.readU16();

            b.isRotatable   = (b.flags & 0x0002) != 0;
            b.isMovable     = (b.flags & 0x0004) != 0;
            b.visible       = (b.flags & 0x0008) != 0;
            b.isControllable = (b.flags & 0x0010) != 0;
            b.isIK          = (b.flags & 0x0020) != 0;
            b.hasAdditionalRotate = (b.flags & 0x0100) != 0;
            b.hasAdditionalLocate = (b.flags & 0x0200) != 0;
            b.appendLocal        = (b.flags & 0x0080) != 0;
            b.hasFixedAxis  = (b.flags & 0x0400) != 0;
            b.hasLocalAxes  = (b.flags & 0x0800) != 0;
            b.transAfterPhys = (b.flags & 0x1000) != 0;

            // bit 0x0001: display connection mode (0=coord offset, 1=bone index)
            if (b.flags & 0x0001) {
                b.displayConnection = reader.readIdx(idxBone);
            } else {
                b.displayOffset = reader.readVec3();
                b.displayConnection = -1;
            }

            if (b.hasAdditionalRotate || b.hasAdditionalLocate) {
                b.appendBoneIdx = reader.readIdx(idxBone);
                b.appendWeight = reader.readF32();
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
                b.ikLinks.resize(linkCount);
                for (int32_t j = 0; j < linkCount; j++) {
                    auto& link = b.ikLinks[j];
                    link.boneIdx = reader.readIdx(idxBone);
                    uint8_t hasLimit = reader.readU8();
                    link.hasLimit = (hasLimit == 1);
                    if (link.hasLimit) {
                        link.limitMin = reader.readVec3();
                        link.limitMax = reader.readVec3();
                    }
                }
            }
        }
    }

    // morphs
    {
        int32_t count = reader.readI32();
        out.morphs.resize(count);
        for (int32_t i = 0; i < count; i++) {
            auto& m = out.morphs[i];
            m.name = reader.readString(encoding);
            m.nameEn = reader.readString(encoding);
            m.category = reader.readU8();
            uint8_t morphType = reader.readU8();
            int32_t dataCount = reader.readI32();

            switch (morphType) {
            case 0: // Group
                m.groupOffsets.resize(dataCount);
                for (int32_t j = 0; j < dataCount; j++) {
                    m.groupOffsets[j].idx = reader.readIdx(idxMorph);
                    m.groupOffsets[j].factor = reader.readF32();
                }
                break;
            case 1: // Vertex
                m.vertexOffsets.resize(dataCount);
                for (int32_t j = 0; j < dataCount; j++) {
                    m.vertexOffsets[j].idx = reader.readIdxUnsigned(idxVertex);
                    m.vertexOffsets[j].offset = reader.readVec3();
                }
                break;
            case 2: // Bone
                m.boneOffsets.resize(dataCount);
                for (int32_t j = 0; j < dataCount; j++) {
                    m.boneOffsets[j].idx = reader.readIdx(idxBone);
                    m.boneOffsets[j].translation = reader.readVec3();
                    m.boneOffsets[j].rotation = reader.readQuat();
                }
                break;
            case 3: case 4: case 5: case 6: case 7: // UV
                m.uvOffsets.resize(dataCount);
                for (int32_t j = 0; j < dataCount; j++) {
                    m.uvOffsets[j].idx = reader.readIdxUnsigned(idxVertex);
                    reader.readFloats(m.uvOffsets[j].offset, 4);
                }
                break;
            case 8: // Material
                m.materialOffsets.resize(dataCount);
                for (int32_t j = 0; j < dataCount; j++) {
                    auto& mm = m.materialOffsets[j];
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
            case 9: // Flip
                for (int32_t j = 0; j < dataCount; j++) {
                    PMXMorph::FlipOffset fo;
                    fo.idx = reader.readIdx(idxMorph);
                    fo.factor = reader.readF32();
                    m.flipOffsets.push_back(fo);
                }
                break;
            case 10: // Impulse
                for (int32_t j = 0; j < dataCount; j++) {
                    PMXMorph::ImpulseOffset io;
                    io.idx = reader.readIdx(idxRigid);
                    io.local = (reader.readU8() != 0);
                    io.velocity = reader.readVec3();
                    io.torque = reader.readVec3();
                    m.impulseOffsets.push_back(io);
                }
                break;
            }
            if (reader.bad()) { err.message = "Morph read failed"; return false; }
        }
    }

    // display frames
    {
        int32_t frameCount = reader.readI32();
        out.displayFrames.resize(frameCount);
        for (int32_t i = 0; i < frameCount; i++) {
            auto& df = out.displayFrames[i];
            df.name = reader.readString(encoding);
            reader.readString(encoding);  // nameEn — skip for now
            df.isSpecial = (reader.readU8() != 0);
            int32_t targetCount = reader.readI32();
            for (int32_t j = 0; j < targetCount; j++) {
                uint8_t type = reader.readU8();
                int32_t idx = (type == 0) ? reader.readIdx(idxBone) : reader.readIdx(idxMorph);
                df.targets.push_back({type, idx});
            }
        }
    }

    // rigid bodies
    {
        int32_t count = reader.readI32();
        out.rigidBodies.resize(count);
        for (int32_t i = 0; i < count; i++) {
            auto& rb = out.rigidBodies[i];
            rb.name = reader.readString(encoding);
            rb.nameEn = reader.readString(encoding);
            rb.boneIdx = reader.readIdx(idxBone);
            rb.group = reader.readU8();
            rb.groupMask = reader.readU16();
            rb.shapeType = reader.readU8();
            rb.size = reader.readVec3();
            rb.position = reader.readVec3();
            float eulerX = reader.readF32(), eulerY = reader.readF32(), eulerZ = reader.readF32();
            rb.rotation = mmp::math::eulerToQuaternionYxz(eulerY, eulerX, eulerZ);
            rb.eulerRotation = btVector3(eulerX, eulerY, eulerZ);
            rb.mass = reader.readF32();
            rb.linearDamping = reader.readF32();
            rb.angularDamping = reader.readF32();
            rb.restitution = reader.readF32();
            rb.friction = reader.readF32();
            rb.mode = reader.readU8();
        }
    }

    // joints
    {
        int32_t count = reader.readI32();
        out.joints.resize(count);
        for (int32_t i = 0; i < count; i++) {
            auto& j = out.joints[i];
            j.name = reader.readString(encoding);
            j.nameEn = reader.readString(encoding);
            j.mode = reader.readU8();
            j.rigidBodyAIdx = reader.readIdx(idxRigid);
            j.rigidBodyBIdx = reader.readIdx(idxRigid);
            j.position = reader.readVec3();
            // PMX stores joint rotation as Euler YXZ radians — convert to quaternion
            float eulerX = reader.readF32(), eulerY = reader.readF32(), eulerZ = reader.readF32();
            j.rotation = mmp::math::eulerToQuaternionYxz(eulerY, eulerX, eulerZ);
            j.linearLowerLimit = reader.readVec3();
            j.linearUpperLimit = reader.readVec3();
            j.angularLowerLimit = reader.readVec3();
            j.angularUpperLimit = reader.readVec3();
            j.springTranslate = reader.readVec3();
            j.springRotate = reader.readVec3();
        }
    }

    // soft body data (PMX 2.1) — skip if present after joints
    if (reader.pos < reader.size) { reader.pos = reader.size; }

    if (reader.bad()) {
        err.message = "Unexpected end of file";return false;
    }return true;
}

ParseResult<PMXData> parse_memory(const uint8_t* data, int size) {
    ParseResult<PMXData> result;
    if (!parse_impl(result.m_value, result.m_error, data, size)) {
        // error already set
    }
    return result;
}

ParseResult<PMXData> parse_file(const char* path) {
    ParseResult<PMXData> result;
    FILE* fp = fopen(path, "rb");
    if (!fp) {
        result.m_error.message = "Cannot open file";
        return result;
    }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz <= 0) {
        fclose(fp);
        result.m_error.message = "Empty or invalid file";
        return result;
    }
    std::vector<uint8_t> buf(sz);
    if (fread(buf.data(), 1, sz, fp) != static_cast<size_t>(sz)) {
        fclose(fp);
        result.m_error.message = "Read failed";
        return result;
    }
    fclose(fp);
    return parse_memory(buf.data(), static_cast<int>(sz));
}

} // namespace mmp::pmx
