// Copyright Epic Games, Inc. All Rights Reserved.
/*===========================================================================
	Generated code exported from UnrealHeaderTool.
	DO NOT modify this manually! Edit the corresponding .h files instead!
===========================================================================*/

#include "UObject/GeneratedCppIncludes.h"
#include "DiscVision/DiscProjectile.h"
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable : 4883)
#endif
PRAGMA_DISABLE_DEPRECATION_WARNINGS
void EmptyLinkFunctionForGeneratedCodeDiscProjectile() {}
// Cross Module References
	DISCVISION_API UClass* Z_Construct_UClass_ADiscProjectile_NoRegister();
	DISCVISION_API UClass* Z_Construct_UClass_ADiscProjectile();
	ENGINE_API UClass* Z_Construct_UClass_AActor();
	UPackage* Z_Construct_UPackage__Script_DiscVision();
// End Cross Module References
	void ADiscProjectile::StaticRegisterNativesADiscProjectile()
	{
	}
	UClass* Z_Construct_UClass_ADiscProjectile_NoRegister()
	{
		return ADiscProjectile::StaticClass();
	}
	struct Z_Construct_UClass_ADiscProjectile_Statics
	{
		static UObject* (*const DependentSingletons[])();
#if WITH_METADATA
		static const UE4CodeGen_Private::FMetaDataPairParam Class_MetaDataParams[];
#endif
		static const FCppClassTypeInfoStatic StaticCppClassTypeInfo;
		static const UE4CodeGen_Private::FClassParams ClassParams;
	};
	UObject* (*const Z_Construct_UClass_ADiscProjectile_Statics::DependentSingletons[])() = {
		(UObject* (*)())Z_Construct_UClass_AActor,
		(UObject* (*)())Z_Construct_UPackage__Script_DiscVision,
	};
#if WITH_METADATA
	const UE4CodeGen_Private::FMetaDataPairParam Z_Construct_UClass_ADiscProjectile_Statics::Class_MetaDataParams[] = {
		{ "IncludePath", "DiscProjectile.h" },
		{ "ModuleRelativePath", "DiscProjectile.h" },
	};
#endif
	const FCppClassTypeInfoStatic Z_Construct_UClass_ADiscProjectile_Statics::StaticCppClassTypeInfo = {
		TCppClassTypeTraits<ADiscProjectile>::IsAbstract,
	};
	const UE4CodeGen_Private::FClassParams Z_Construct_UClass_ADiscProjectile_Statics::ClassParams = {
		&ADiscProjectile::StaticClass,
		"Engine",
		&StaticCppClassTypeInfo,
		DependentSingletons,
		nullptr,
		nullptr,
		nullptr,
		UE_ARRAY_COUNT(DependentSingletons),
		0,
		0,
		0,
		0x009000A4u,
		METADATA_PARAMS(Z_Construct_UClass_ADiscProjectile_Statics::Class_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UClass_ADiscProjectile_Statics::Class_MetaDataParams))
	};
	UClass* Z_Construct_UClass_ADiscProjectile()
	{
		static UClass* OuterClass = nullptr;
		if (!OuterClass)
		{
			UE4CodeGen_Private::ConstructUClass(OuterClass, Z_Construct_UClass_ADiscProjectile_Statics::ClassParams);
		}
		return OuterClass;
	}
	IMPLEMENT_CLASS(ADiscProjectile, 2326596464);
	template<> DISCVISION_API UClass* StaticClass<ADiscProjectile>()
	{
		return ADiscProjectile::StaticClass();
	}
	static FCompiledInDefer Z_CompiledInDefer_UClass_ADiscProjectile(Z_Construct_UClass_ADiscProjectile, &ADiscProjectile::StaticClass, TEXT("/Script/DiscVision"), TEXT("ADiscProjectile"), false, nullptr, nullptr, nullptr);
	DEFINE_VTABLE_PTR_HELPER_CTOR(ADiscProjectile);
PRAGMA_ENABLE_DEPRECATION_WARNINGS
#ifdef _MSC_VER
#pragma warning (pop)
#endif
