#pragma once

#include <unordered_set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
    struct Options {
        int logLevel = IDebugLog::kLevel_Message;

        float interiorDrawDistance = 30.f;
        float exteriorDrawDistance = 50.f;

        bool wireframe = true;

        int numSphereSegments = 12;
        int numCapsuleSegments = 12;

        bool inflateByConvexRadius = true;
        bool dedupConvexVertices = true;
        float dedupConvexVerticesThreshold = 0.001f;
        float dedupConvexVerticesThresholdCleanup = 1.f;
        bool duplicatePlanarShapeVertices = true;

        float constraintPivotSphereRadius = 0.03f;

        bool resetOnToggle = true;

        bool drawActiveIslands = true;
        bool drawInactiveIslands = true;
        bool drawFixedIsland = true;
        bool drawPhantoms = true;
        bool drawConstraints = true;
        bool drawPlayerCharacterController = true;

        std::unordered_set<int> ignoreLayers{};
        std::unordered_map<int, NiColorA> layerColors{};

        NiColorA defaultColor = { 1.f, 1.f, 1.f, 1.f };
        NiColorA phantomColor = { 1.f, 1.f, 0.f, 1.f };
        NiColorA proxyCharControllerColor = { 1.f, 0.f, 0.f, 1.f };
        NiColorA ragdollConstraintColor = { 0.f, 0.f, 1.f, 1.f };
        NiColorA hingeConstraintColor = { 0.f, 0.f, 1.f, 1.f };
        float fixedObjectDimFactor = 0.3f;
    };
    extern Options options; // global object containing options


    // Fills Options struct from INI file
    bool ReadConfigOptions();
    bool ReloadIfModified();

    const std::string & GetConfigPath();

    std::string GetConfigOption(const char * section, const char * key);

    bool GetConfigOptionDouble(const char *section, const char *key, double *out);
    bool GetConfigOptionFloat(const char *section, const char *key, float *out);
    bool GetConfigOptionInt(const char *section, const char *key, int *out);
    bool GetConfigOptionUInt64Hex(const char *section, const char *key, UInt64 *out);
    bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
