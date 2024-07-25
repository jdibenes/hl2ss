
#pragma once

#ifdef HL2SS_PLUGIN
#define HL2SS_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define HL2SS_PLUGIN_EXPORT 
#endif
