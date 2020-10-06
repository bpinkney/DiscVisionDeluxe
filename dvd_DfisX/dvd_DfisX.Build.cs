// dvd_DfisX.Build.cs

using UnrealBuildTool;

public class dvd_DfisX : ModuleRules
{
    public dvd_DfisX(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            });
    }
}