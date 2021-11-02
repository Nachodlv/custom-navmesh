#include "INNModule.h"

#define LOCTEXT_NAMESPACE "FPMTAIModule"

class FNNModule : public INNModule
{
public:

	/** IModuleInterface implementation */
	void StartupModule() override
	{
		// This is loaded upon first request
	}

	void ShutdownModule() override
	{
	}
};

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FNNModule, NachoNavmesh);
