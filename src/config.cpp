#include <chrono>
#include <filesystem>

#include "config.h"
#include "utils.h"


namespace Config {
    // Define extern options
    Options options;

    bool ReadFloat(const std::string &name, float &val)
    {
        if (!GetConfigOptionFloat("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read float config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadDouble(const std::string &name, double &val)
    {
        if (!GetConfigOptionDouble("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read double config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadBool(const std::string &name, bool &val)
    {
        if (!GetConfigOptionBool("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read bool config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadInt(const std::string &name, int &val)
    {
        if (!GetConfigOptionInt("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read int config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadUInt64Hex(const std::string &name, UInt64 &val)
    {
        if (!GetConfigOptionUInt64Hex("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read hex uint64 config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadString(const std::string &name, std::string &val)
    {
        std::string	data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read str config option: %s", name.c_str());
            return false;
        }

        val = data;
        return true;
    }

    bool ReadVector(const std::string &name, NiPoint3 &vec)
    {
        if (!ReadFloat(name + "X", vec.x)) return false;
        if (!ReadFloat(name + "Y", vec.y)) return false;
        if (!ReadFloat(name + "Z", vec.z)) return false;

        return true;
    }

    bool ReadStringSet(const std::string &name, std::set<std::string, std::less<>> &val)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read StringSet config option: %s", name.c_str());
            return false;
        }

        val = SplitStringToSet(data, ',');
        return true;
    }

    bool ReadIntArray(const std::string &name, std::vector<int> &val)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read int array config option: %s", name.c_str());
            return false;
        }

        val.clear(); // first empty the set, since we will be reading into it

        std::vector<std::string> stringList = SplitString(data, ',');
        for (const std::string &str : stringList) {
            val.push_back(std::stoi(str));
        }
        return true;
    }

    bool ReadIntSet(const std::string &name, std::unordered_set<int> &val)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read int set config option: %s", name.c_str());
            return false;
        }

        val.clear(); // first empty the set, since we will be reading into it

        std::vector<std::string> stringList = SplitString(data, ',');
        for (const std::string &str : stringList) {
            val.insert(std::stoi(str));
        }
        return true;
    }

    NiColorA ConvertHexToColor(UInt32 hex)
    {
        NiColorA color;
        color.r = ((hex >> 16) & 0xff) / 255.f;
        color.g = ((hex >> 8) & 0xff) / 255.f;
        color.b = (hex & 0xff) / 255.f;
        color.a = 1.f;
        return color;
    }

    bool ReadColor(const std::string &name, NiColorA &color)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read color config option: %s", name.c_str());
            return false;
        }

        UInt32 hex = std::stoi(data, nullptr, 16);

        color = ConvertHexToColor(hex);
        return true;
    }

    bool ReadLayerColors(const std::string &name, std::unordered_map<int, NiColorA> &colors)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read layer color config option: %s", name.c_str());
            return false;
        }

        colors.clear(); // first empty the set, since we will be reading into it

        // Layer colors are stored as a comma-separated list of layer index:color pairs, where the color is the hex value of the color
        std::vector<std::string> stringList = SplitString(data, ',');
        for (const std::string &str : stringList) {
            std::vector<std::string> layerColor = SplitString(str, ':');
            if (layerColor.size() != 2) {
                _WARNING("Invalid layer color config option: %s", str.c_str());
                return false;
            }

            int layer = std::stoi(layerColor[0]);
            UInt32 color = std::stoi(layerColor[1], nullptr, 16);

            colors[layer] = ConvertHexToColor(color);
        }

        return true;
    }

    bool ReadConfigOptions()
    {
        if (!ReadInt("logLevel", options.logLevel)) return false;

        if (!ReadFloat("interiorDrawDistance", options.interiorDrawDistance)) return false;
        if (!ReadFloat("exteriorDrawDistance", options.exteriorDrawDistance)) return false;

        if (!ReadBool("wireframe", options.wireframe)) return false;

        if (!ReadInt("numSphereSegments", options.numSphereSegments)) return false;
        if (!ReadInt("numCapsuleSegments", options.numCapsuleSegments)) return false;

        if (!ReadBool("inflateByConvexRadius", options.inflateByConvexRadius)) return false;
        if (!ReadBool("dedupConvexVertices", options.dedupConvexVertices)) return false;
        if (!ReadFloat("dedupConvexVerticesThreshold", options.dedupConvexVerticesThreshold)) return false;
        if (!ReadFloat("dedupConvexVerticesThresholdCleanup", options.dedupConvexVerticesThresholdCleanup)) return false;
        if (!ReadBool("duplicatePlanarShapeVertices", options.duplicatePlanarShapeVertices)) return false;

        if (!ReadFloat("constraintPivotSphereRadius", options.constraintPivotSphereRadius)) return false;
        if (!ReadFloat("centerOfMassSphereRadius", options.centerOfMassSphereRadius)) return false;

        if (!ReadBool("resetOnToggle", options.resetOnToggle)) return false;

        if (!ReadBool("drawActiveIslands", options.drawActiveIslands)) return false;
        if (!ReadBool("drawInactiveIslands", options.drawInactiveIslands)) return false;
        if (!ReadBool("drawFixedIsland", options.drawFixedIsland)) return false;
        if (!ReadBool("drawPhantoms", options.drawPhantoms)) return false;
        if (!ReadBool("drawConstraints", options.drawConstraints)) return false;
        if (!ReadBool("drawPlayerCharacterController", options.drawPlayerCharacterController)) return false;
        if (!ReadBool("drawCenterOfMass", options.drawCenterOfMass)) return false;

        if (!ReadIntSet("ignoreLayers", options.ignoreLayers)) return false;

        if (!ReadLayerColors("layerColors", options.layerColors)) return false;

        if (!ReadColor("defaultColor", options.defaultColor)) return false;
        if (!ReadColor("phantomColor", options.phantomColor)) return false;
        if (!ReadColor("proxyCharControllerColor", options.proxyCharControllerColor)) return false;
        if (!ReadColor("ragdollConstraintColor", options.ragdollConstraintColor)) return false;
        if (!ReadColor("hingeConstraintColor", options.hingeConstraintColor)) return false;
        if (!ReadColor("centerOfMassColor", options.centerOfMassColor)) return false;
        if (!ReadFloat("fixedObjectDimFactor", options.fixedObjectDimFactor)) return false;

        return true;
    }

    bool ReloadIfModified()
    {
        namespace fs = std::filesystem;

        static long long lastModifiedConfigTime = 0;

        const std::string &path = GetConfigPath();
        auto ftime = fs::last_write_time(path);
        auto time = ftime.time_since_epoch().count();
        if (time > lastModifiedConfigTime) {
            lastModifiedConfigTime = time;

            // Reload config if file has been modified since we last read it
            if (Config::ReadConfigOptions()) {
                _MESSAGE("Successfully reloaded config parameters");
            }
            else {
                _WARNING("[WARNING] Failed to reload config options");
            }

            return true;
        }

        return false;
    }

    const std::string & GetConfigPath()
    {
        static std::string s_configPath;

        if (s_configPath.empty()) {
            std::string	runtimePath = GetRuntimeDirectory();
            if (!runtimePath.empty()) {
                s_configPath = runtimePath + "Data\\SKSE\\Plugins\\collviz_vr.ini";

                _MESSAGE("config path = %s", s_configPath.c_str());
            }
        }

        return s_configPath;
    }

    std::string GetConfigOption(const char *section, const char *key)
    {
        std::string	result;

        const std::string & configPath = GetConfigPath();
        if (!configPath.empty()) {
            char	resultBuf[256];
            resultBuf[0] = 0;

            UInt32	resultLen = GetPrivateProfileString(section, key, NULL, resultBuf, sizeof(resultBuf), configPath.c_str());

            result = resultBuf;
        }

        return result;
    }

    bool GetConfigOptionDouble(const char *section, const char *key, double *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stod(data);
        return true;
    }

    bool GetConfigOptionFloat(const char *section, const char *key, float *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stof(data);
        return true;
    }

    bool GetConfigOptionInt(const char *section, const char *key, int *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stoi(data);
        return true;
    }

    bool GetConfigOptionUInt64Hex(const char *section, const char *key, UInt64 *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stoull(data, nullptr, 16);
        return true;
    }

    bool GetConfigOptionBool(const char *section, const char *key, bool *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        int val = std::stoi(data);
        if (val == 1) {
            *out = true;
            return true;
        }
        else if (val == 0) {
            *out = false;
            return true;
        }
        else {
            return false;
        }
    }
}
