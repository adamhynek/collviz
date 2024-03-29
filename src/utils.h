#pragma once

#include <set>

#include "skse64/NiGeometry.h"
#include "skse64/GameReferences.h"
#include "skse64/PapyrusSpell.h"

#include "havok.h"


typedef bool(*IAnimationGraphManagerHolder_GetGraphVariableInt)(IAnimationGraphManagerHolder *_this, const BSFixedString& a_variableName, SInt32& a_out);
typedef bool(*IAnimationGraphManagerHolder_GetGraphVariableBool)(IAnimationGraphManagerHolder* _this, const BSFixedString& a_variableName, bool& a_out);
typedef bool(*_SpellItem_IsTwoHanded)(SpellItem *_this);

inline UInt64* get_vtbl(void* object) { return *((UInt64**)object); }

template<typename T>
inline T get_vfunc(void* object, UInt64 index) {
    UInt64* vtbl = get_vtbl(object);
    return (T)(vtbl[index]);
}

inline NiTransform InverseTransform(const NiTransform &t) { NiTransform inverse; t.Invert(inverse); return inverse; }
inline float VectorLengthSquared(const NiPoint3 &vec) { return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; }
inline float VectorLength(const NiPoint3 &vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float lerp(float a, float b, float t) { return a * (1.0f - t) + b * t; }
inline NiPoint3 lerp(const NiPoint3 &a, const NiPoint3 &b, float t) { return a * (1.0f - t) + b * t; }
inline NiPoint3 ForwardVector(const NiMatrix33 &r) { return { r.data[0][1], r.data[1][1], r.data[2][1] }; }
inline float DotProduct(const NiPoint3 &vec1, const NiPoint3 &vec2) { return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; }
inline NiPoint3 VectorNormalized(const NiPoint3 &vec) { float length = VectorLength(vec); return length > 0.0f ? vec / length : NiPoint3(); }

inline void ltrim(std::string &s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !::isspace(ch); })); }
inline void rtrim(std::string &s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !::isspace(ch); }).base(), s.end()); }
inline void trim(std::string &s) { ltrim(s); rtrim(s); }
std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim);
std::vector<std::string> SplitString(const std::string &s, char delim);

inline NiPoint3 HkVectorToNiPoint(const hkVector4 &vec) { return { vec.getQuad().m128_f32[0], vec.getQuad().m128_f32[1], vec.getQuad().m128_f32[2] }; }
inline hkVector4 NiPointToHkVector(const NiPoint3 &pt) { return { pt.x, pt.y, pt.z, 0 }; };

NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2);
NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle);
void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat);
NiTransform hkTransformToNiTransform(const hkTransform &t, float scale = 1.f, bool useHavokScale = true);

inline UInt32 GetCollisionLayer(UInt32 collisionFilterInfo) { return collisionFilterInfo & 0x7f; }
inline void SetCollisionLayer(hkUint32 &collisionFilterInfo, UInt32 layer) {
    collisionFilterInfo &= ~(0x7f); // zero out layer
    collisionFilterInfo |= (layer & 0x7f); // set layer to the same as a dead ragdoll
}
inline UInt32 GetCollisionLayer(const hkpRigidBody *rigidBody) { return GetCollisionLayer(rigidBody->getCollisionFilterInfo()); }
inline void SetCollisionLayer(hkpRigidBody *rigidBody, UInt32 layer) { return SetCollisionLayer(rigidBody->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo, layer); }
