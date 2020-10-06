// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class DiscVisionDeluxeUE : ModuleRules
{
	public DiscVisionDeluxeUE(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true

		// add libs        
        PrivateIncludePaths.Add( "../../dvd_DvisEst/lib/eigen" );

        // add headers
        PrivateIncludePaths.Add( "../../common/inc");
        PrivateIncludePaths.Add( "DiscVisionDeluxeUE/DfisX");
        //PrivateIncludePaths.Add( "../../dvd_DfisX/Private");

        
        // add src files
        //PublicDependencyModuleNames.Add("dvd_DfisX");
        //PublicDependencyModuleNames.Add( "dvd_DfisX");


		
	}
}
