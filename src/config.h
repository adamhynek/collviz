#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		int logLevel = IDebugLog::kLevel_Message;

		float drawDistance = 50.f;
		bool wireframe = true;
		bool inflateByConvexRadius = true;
		bool dedupConvexVertices = true;
		float dedupConvexVerticesThreshold = 1.f;
		bool duplicatePlanarShapeVertices = true;
		bool resetOnToggle = true;
		NiColorA dynamicColor = { 1.f, 1.f, 1.f, 1.f };
		NiColorA fixedColor = { 0.3f, 0.3f, 0.3f, 1.f };
	};
	extern Options options; // global object containing options


	// Fills Options struct from INI file
	bool ReadConfigOptions();

	const std::string & GetConfigPath();

	std::string GetConfigOption(const char * section, const char * key);

	bool GetConfigOptionDouble(const char *section, const char *key, double *out);
	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionInt(const char *section, const char *key, int *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
