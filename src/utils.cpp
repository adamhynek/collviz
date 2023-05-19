#include <sstream>

#include "skse64/GameRTTI.h"

#include "utils.h"
#include "offsets.h"


NiTransform GetLocalTransform(NiAVObject *node, const NiTransform &worldTransform)
{
    NiPointer<NiNode> parent = node->m_parent;
    if (parent) {
        NiTransform inverseParent = InverseTransform(node->m_parent->m_worldTransform);
        return inverseParent * worldTransform;
    }
    return worldTransform;
}

void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform)
{
    // Given world transform, set the necessary local transform
    node->m_localTransform = GetLocalTransform(node, worldTransform);
}

bool GetAnimVariableBool(Actor *actor, BSFixedString &variableName)
{
    IAnimationGraphManagerHolder *animGraph = &actor->animGraphHolder;
    UInt64 *vtbl = *((UInt64 **)animGraph);
    bool var = false;
    ((IAnimationGraphManagerHolder_GetGraphVariableBool)(vtbl[0x12]))(animGraph, variableName, var);
    return var;
}

bool IsDualCasting(Actor *actor)
{
    static BSFixedString animVarName("IsCastingDual");
    return GetAnimVariableBool(actor, animVarName);
}

bool IsCastingRight(Actor *actor)
{
    static BSFixedString animVarName("IsCastingRight");
    return GetAnimVariableBool(actor, animVarName);
}

bool IsCastingLeft(Actor *actor)
{
    static BSFixedString animVarName("IsCastingLeft");
    return GetAnimVariableBool(actor, animVarName);
}

NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2)
{
    NiPoint3 result;
    result.x = vec1.y * vec2.z - vec1.z * vec2.y;
    result.y = vec1.z * vec2.x - vec1.x * vec2.z;
    result.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return result;
}

NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle)
{
    // Rodrigues' rotation formula
    float cosTheta = cosf(angle);
    return vector * cosTheta + (CrossProduct(axis, vector) * sinf(angle)) + axis * DotProduct(axis, vector) * (1.0f - cosTheta);
}

void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat)
{
    hkVector4 col0, col1, col2;
    hkMat.getCols(col0, col1, col2);

    niMat.data[0][0] = col0(0);
    niMat.data[1][0] = col0(1);
    niMat.data[2][0] = col0(2);

    niMat.data[0][1] = col1(0);
    niMat.data[1][1] = col1(1);
    niMat.data[2][1] = col1(2);

    niMat.data[0][2] = col2(0);
    niMat.data[1][2] = col2(1);
    niMat.data[2][2] = col2(2);
}

NiTransform hkTransformToNiTransform(const hkTransform &t, float scale, bool useHavokScale)
{
    NiTransform out;
    out.pos = HkVectorToNiPoint(t.m_translation) * (useHavokScale ? *g_inverseHavokWorldScale : 1.f);
    HkMatrixToNiMatrix(t.m_rotation, out.rot);
    out.scale = scale;
    return out;
}

std::vector<std::string> SplitString(const std::string &s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim)) {
        trim(item);
        result.push_back(item);
    }

    return result;
}

std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim)
{
	std::set<std::string, std::less<>> result;
	std::stringstream ss(s);
	std::string item;

	while (getline(ss, item, delim)) {
		trim(item);
		result.insert(item);
	}

	return result;
}

