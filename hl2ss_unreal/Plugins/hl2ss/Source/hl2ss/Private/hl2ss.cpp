// Copyright Epic Games, Inc. All Rights Reserved.

#include "hl2ss.h"

#define LOCTEXT_NAMESPACE "Fhl2ssModule"

void Fhl2ssModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void Fhl2ssModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(Fhl2ssModule, hl2ss)