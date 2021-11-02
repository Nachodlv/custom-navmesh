// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class NachoNavmesh : ModuleRules
{
	public NachoNavmesh(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
		);


		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
		);

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"AIModule",
			"NavigationSystem",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// TODO (Ignacio) we might want to remove some of these, I really don't know which one is necessary
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"Slate",
			"SlateCore",
		});

		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
		);
	}
}
