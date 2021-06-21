// Fill out your copyright notice in the Description page of Project Settings.


#include "BagMakerWidget.h"




 void UBagMakerWidget::NativeConstruct()
 {


    //TArray<FListDisc> list_disc_array= {};


//list_disc_array.Add (testlistdisc);

//TArray<FListDisc> list_disc_array= TArray<FListDisc>();
    // list_disc_array= {};
//list_disc_array.add(testlistdisc);
 	
     for (int i = 0; i < DfisX::disc_object_array.size(); ++i)
     {
     	FListDisc temp_list_disc = FListDisc();



		temp_list_disc.mold_name = DfisX::disc_object_array[i].mold_name;	
		temp_list_disc.manufacturer = DfisX::disc_object_array[i].manufacturer;	
		temp_list_disc.disc_type = DfisX::disc_object_array[i].disc_type;	

		list_disc_array.Add (temp_list_disc);

     
     }
          Super::NativeConstruct();
}


