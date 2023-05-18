#pragma once

#include "havok.h"

#include "skse64_common/Relocation.h"
#include "skse64/GameReferences.h"
#include "skse64/NiNodes.h"

#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>

// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
extern RelocPtr<float> g_havokWorldScale;
extern RelocPtr<float> g_inverseHavokWorldScale;
extern RelocPtr<bool> g_refractionDebug;
extern RelocPtr<int> g_currentFrameCounter;

typedef bhkWorld *(*_GetHavokWorldFromCell)(TESObjectCELL *cell);
extern RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell;

typedef NiAVObject *(*_GetNodeFromCollidable)(const hkpCollidable *a_collidable);
extern RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable;

typedef void(*_hkpConvexVerticesShape_getOriginalVertices)(hkpConvexVerticesShape *_this, hkArray<hkVector4> &vertices);
extern RelocAddr<_hkpConvexVerticesShape_getOriginalVertices> hkpConvexVerticesShape_getOriginalVertices;

typedef void(*_NiCamera_FinishAccumulatingPostResolveDepth)(NiCamera *camera, void *shaderAccumulator, UInt32 flags);
extern RelocAddr<_NiCamera_FinishAccumulatingPostResolveDepth> NiCamera_FinishAccumulatingPostResolveDepth;
