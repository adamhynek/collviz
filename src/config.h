#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
    struct Options {
        int logLevel = IDebugLog::kLevel_Message;

        float drawDistance = 50.f;

        bool wireframe = true;

        int numSphereSegments = 12;
        int numCapsuleSegments = 12;

        bool inflateByConvexRadius = true;
        bool dedupConvexVertices = true;
        float dedupConvexVerticesThreshold = 0.001f;
        float dedupConvexVerticesThresholdCleanup = 1.f;
        bool duplicatePlanarShapeVertices = true;

        bool resetOnToggle = true;

        UInt64 drawLayersBitfield = 0xffffffffbfffffff; // everything except the charcontroller layer
        bool drawActiveIslands = true;
        bool drawInactiveIslands = true;
        bool drawFixedIsland = true;

        NiColorA dynamicColor = { 1.f, 1.f, 1.f, 1.f };
        NiColorA fixedColor = { 0.3f, 0.3f, 0.3f, 1.f };
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
