// Fill out your copyright notice in the Description page of Project Settings.

#include "DiscProjectile.h"
#include <math.h>

// needed for custom_disc IO
// load files
#include <fstream>      // std::ofstream
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::ostringstrea

#include "HAL/FileManagerGeneric.h"

// find list of files:
/*#define WIN32_LEAN_AND_MEAN
#include "Windows/WindowsSystemIncludes.h"
#include <windows.h>
#include <tchar.h>
#include <stdio.h>*/
// end custom_disc IO

//#include <filesystem>
//namespace fs = std::experimental::filesystem;
//#include "DfisX\DfisX.hpp"
//#include "Components/SphereComponent.h"
//#include "GameFramework/ProjectileMovementComponent.h"



// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//kill_control_with_delay();

	

	//Dies after 10m seconds.
	InitialLifeSpan = 600.0f;
	
	
	
}

// Called when the game starts or when spawned
void ADiscProjectile::BeginPlay()
{
	Super::BeginPlay();

	//custom_disc_test_save_and_load();

 //kill_control_with_delay();

}
/*
void ADiscProjectile::initialize_custom_disc(const FCustom_disc & custom_disc)
{
	/*
	FString 		mold_name		="None";					//for lookup with disc params
	enum_disc_form 	mold_form		=enum_disc_form::FRISBEE; 	//see above
	float			disc_mass  		=0.175						//mass in kg
	float 			disc_wear		=0.0;   					//0..1
	FColor			base_colour		=FColor::Red;				//main colour of plastic
	FColor			secondary_colour=FColor::Blue;				//colour of patterning on disc
	FColor			rim_colour		=FColor::Black;				//colour of rim
	FString			decal_path		="None.png";				//filepath to decal texture
	int 			pattern_enum	=1;							//TODO change to pattern enum when implemented
	FString			player_name     ="";					//the player who created disc
	FDateTime		date_created; 								//date of creation
	
	if (custom_disc.player_name=="")   //not form a player bag ; this would be if this disc is initialized from a throw generator or somesuch
	{



	} else  // normal case
	{

	set_disc_mesh_and_mass		(custom_disc.mold_form,custom_disc.disc_mass);
	set_disc_wear				(custom_disc.disc_wear);
	set_disc_base_colour 		(custom_disc.base_colour);
	set_disc_secondary_colour 	(custom_disc.secondary_colour);
	set_disc_rim_colour  		(custom_disc.rim_colour);
	set_disc_texture			(custom_disc.decal_path);
	set_disc_pattern			(custom_disc.pattern_enum,custom_disc.pattern_seed);
	//set_disc_pattern (TODO)

	}

}
*/
// Called every frame
void ADiscProjectile::Tick(float DeltaTime)
{
	
	
	Super::Tick(DeltaTime);

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, FString::Printf(TEXT("Disc Location is: %s"), *disc_position.ToString()));
}

FString ADiscProjectile::custom_disc_get_save_path(void)
{
	std::string path = std::string(TCHAR_TO_UTF8(*FPaths::ProjectContentDir())) + "/custom_disc_saves/";
	return FString(path.c_str());
}

// local save/load functions (hopefully multiple threads won't make this fight itself...)
void ADiscProjectile::custom_disc_save(FCustom_disc * custom_disc)
{
	FString custom_disc_name = ADiscProjectile::custom_disc_generate_name_string(custom_disc);

	std::string custom_disc_name_std = std::string(TCHAR_TO_UTF8(*custom_disc_name));

	std::ofstream custom_disc_save_file;
	// make sure to overwrite the previous contents
	std::string save_filename = std::string(TCHAR_TO_UTF8(*ADiscProjectile::custom_disc_get_save_path())) + custom_disc_name_std + ".csv";

	custom_disc_save_file.open(save_filename, std::ofstream::trunc);


	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString("SAVE DISC!"));
	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, custom_disc_name);
	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,  FString(save_filename.c_str()));

	// seems like Unreal structs don't write binary correctly..... sigh
	// just do it as a csv then
	std::string delimiter = ";";

	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->player_name))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->mold_name))) << delimiter;
	custom_disc_save_file << (int)custom_disc->mold_form << delimiter;
	custom_disc_save_file << custom_disc->disc_mass << delimiter;
	custom_disc_save_file << custom_disc->disc_wear << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->base_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->secondary_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->rim_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->decal_path))) << delimiter;
	custom_disc_save_file << (int)custom_disc->pattern_enum << delimiter;
	custom_disc_save_file << custom_disc->pattern_seed << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->date_created.ToString()))) << delimiter;

	custom_disc_save_file.close();
}

// generate name string from struct components
FString ADiscProjectile::custom_disc_generate_name_string(FCustom_disc * custom_disc)
{
	FString output;

	// encode the enums as ints directly for now
	// use default float precision for now
/*	player name           
	+mold name           
	+mass                      
	+pattern type         
	+pattern seed        
	+base colour*/

	std::ostringstream ss;
	ss << std::string(TCHAR_TO_UTF8(*(custom_disc->player_name))) << "_";
	ss << std::string(TCHAR_TO_UTF8(*(custom_disc->mold_name))) << "_";
	ss << custom_disc->disc_mass << "_";
	ss << (int)custom_disc->pattern_enum << "_";
	ss << custom_disc->pattern_seed << "_";
	ss << std::string(TCHAR_TO_UTF8(*(custom_disc->base_colour.ToString()))) << "_";
	
	std::string output_std = ss.str();

	// remove illegal characters
	std::string invalid = "<>=:\"/\\|?*()[] ";

	for(std::string::size_type i = 0; i < invalid.size(); ++i) 
	{
		char invalid_char = invalid[i];
		std::replace( output_std.begin(), output_std.end(), invalid_char, '-'); // replace all 'x' to 'y'
	}

	output = FString(output_std.c_str());

	return output;
}

// load by name string match, true if matching disc was found
bool ADiscProjectile::custom_disc_load(FString custom_disc_name, FCustom_disc * custom_disc)
{
	/*	FString 		mold_name		="None";					//for lookup with disc params
	enum_disc_form 	mold_form		=enum_disc_form::FRISBEE; 	//see above
	float			disc_mass  		=0.175;						//mass in kg
	float 			disc_wear		=0.0;   					//0..1
	FLinearColor	base_colour		=FLinearColor::Red;			//main colour of plastic
	FLinearColor	secondary_colour=FLinearColor::Blue;		//colour of patterning on disc
	FLinearColor	rim_colour		=FLinearColor::Black;		//colour of rim
	FString			decal_path		="None.png";				//filepath to decal texture
	enum_pattern_type pattern_enum = enum_pattern_type::SPIRAL;	//TODO change to pattern enum when implemented
	int 			pattern_seed	=0;							//
	FString			player_name		="";						//the player who created disc
	FDateTime		date_created; */
/*	std::string delimiter = ";";


	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->mold_name))) << delimiter;
	custom_disc_save_file << (int)custom_disc->mold_form << delimiter;
	custom_disc_save_file << custom_disc->disc_mass << delimiter;
	custom_disc_save_file << custom_disc->disc_wear << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->base_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->secondary_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->rim_colour.ToString()))) << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->decal_path))) << delimiter;
	custom_disc_save_file << (int)custom_disc->pattern_enum << delimiter;
	custom_disc_save_file << custom_disc->pattern_seed << delimiter;
	custom_disc_save_file << std::string(TCHAR_TO_UTF8(*(custom_disc->date_created.ToString()))) << delimiter;*/

	std::string token;                      // This will store your tokens
	std::string custom_disc_name_std = std::string(TCHAR_TO_UTF8(*custom_disc_name));
	std::string load_filename = std::string(TCHAR_TO_UTF8(*ADiscProjectile::custom_disc_get_save_path())) + custom_disc_name_std;
	std::ifstream load_file(load_filename.c_str());

	char delimiter = ';';
	int line_num = 0;

	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Black, FString(load_filename.c_str()));

	const uint8_t line_total = 12;

	while(std::getline(load_file, token, delimiter))
	{
		GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Black, FString(token.c_str()));
		switch(line_num)
		{
			case 0:
				custom_disc->player_name 		= FString(token.c_str());
				break;
			case 1:
				custom_disc->mold_name 			= FString(token.c_str());
				break;
			case 2:
				custom_disc->mold_form 			= static_cast<enum_disc_form>(std::stoi(token.c_str()));
				break;
			case 3:
				custom_disc->disc_mass 			= std::stof(token.c_str());
				break;
			case 4:
				custom_disc->disc_wear 			= std::stof(token.c_str());
				break;
			case 5:
			{
				FLinearColor colour;
				colour.InitFromString(FString(token.c_str()));
				custom_disc->base_colour 		= colour;
				break;
			}
			case 6:
			{
				FLinearColor colour;
				colour.InitFromString(FString(token.c_str()));
				custom_disc->secondary_colour 	= colour;
				break;
			}
			case 7:
			{
				FLinearColor colour;
				colour.InitFromString(FString(token.c_str()));
				custom_disc->rim_colour 		= colour;
				break;
			}
			case 8:
				custom_disc->decal_path 		= FString(token.c_str());
				break;
			case 9:
				custom_disc->pattern_enum 		= static_cast<enum_pattern_type>(std::stoi(token.c_str()));
				break;
			case 10:
				custom_disc->pattern_seed 		= std::stof(token.c_str());
				break;
			case 11:
			{
				FDateTime date_created;
				FDateTime::Parse(FString(token.c_str()), date_created);
				custom_disc->date_created 		= date_created;
				break;
			}
		}

	    line_num++;
	}

	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Black, FString("Found Lines:"));
	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Black, FString::FromInt(line_num));
	return (line_num == line_total);
}

//void custom_disc_load_all(TArray<FCustom_disc> custom_disc_array);
void ADiscProjectile::custom_disc_load_disc_names(TArray<FString> * custom_disc_names)
{
	std::string load_path = std::string(TCHAR_TO_UTF8(*ADiscProjectile::custom_disc_get_save_path()));

	FFileManagerGeneric FileMgr;
	FileMgr.SetSandboxEnabled(true);// don't ask why, I don't know :P
	FString extension("*.csv"); // May be "" (empty string) to search all files
	FString search_path(FPaths::Combine(FString(load_path.c_str()), *extension));
	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow, search_path);

	FileMgr.FindFiles(*custom_disc_names, *search_path, 
                                  true,  // to list files
                                  false); // to skip directories

	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red, FString("Found Files:"));
	for (auto f : *custom_disc_names)
	{
		FString filename(f);
		GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red, filename);
	}
}

void ADiscProjectile::custom_disc_test_save_and_load(void)
{
	// define disc
	FCustom_disc custom_disc;
	custom_disc.mold_name 				= "Tossed Salad";
	custom_disc.mold_form 				= enum_disc_form::DRIVER;
	custom_disc.disc_mass  				= 0.175;
	custom_disc.disc_wear				= 0.0;
	custom_disc.base_colour				= FLinearColor::Yellow;
	custom_disc.secondary_colour 		= FLinearColor::Red;
	custom_disc.rim_colour				= FLinearColor::Red;
	custom_disc.decal_path				= "None.png";
	custom_disc.pattern_enum 			= enum_pattern_type::SPIRAL;
	custom_disc.pattern_seed			= 0;
	custom_disc.player_name				= "Mr. Bojangles";
	custom_disc.date_created    		= FDateTime::Now();

	// get name string just for fun
	FString custom_disc_name = ADiscProjectile::custom_disc_generate_name_string(&custom_disc);

	// save to file
	ADiscProjectile::custom_disc_save(&custom_disc);

	// get list of saved discs
	TArray<FString> custom_disc_names;
	custom_disc_load_disc_names(&custom_disc_names);

	// load disc by name [0]
	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString("Found Files:"));
	for (auto f : custom_disc_names)
	{
		FString filename(f);
		GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, filename);
	}
	if(custom_disc_names.Num() > 0 && custom_disc_names.IsValidIndex(0))
	{

		FCustom_disc custom_disc_loaded;
		bool load_worked = custom_disc_load(custom_disc_names[0], &custom_disc_loaded);

		// Now change the name and re-save to make sure we loaded correctly
		if(load_worked)
		{
			custom_disc_loaded.player_name = "Sir Jangles the Third";
			// save again to file
			ADiscProjectile::custom_disc_save(&custom_disc_loaded);
		}
	}
}



