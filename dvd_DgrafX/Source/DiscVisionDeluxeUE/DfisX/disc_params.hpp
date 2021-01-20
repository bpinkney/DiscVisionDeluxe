
#pragma once

#include "DfisX.hpp"

namespace DfisX
{



  // For now, this is just a hard-coded mapping to fixed discs from test data
  // and using the ordering of the 'DiscIndex' enums
  // duplicate entries are just copied from the destroyer stats

  const std::vector<DfisX::Disc_Model> disc_object_array = 
  {
    // 0
    Disc_Model
    { // AAADiscs Mr Brick (Champion)
      /*.mold_name =*/          "Mr Brick",
      /*.manufacturer =*/       "AAADiscs",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               3.00000,
      /*.radius =*/             0.12000,
      /*.rim_width =*/          0.00000,
      /*.thickness =*/          0.07000,
      /*.rim_depth =*/          0.00000,
      /*.rim_camber_height =*/  0.00000,
      /*.dome_height =*/        0.00000,
    },
    // 1
    Disc_Model
    { // Daredevil_Discs Polar Bear (Grip Performance)
      /*.mold_name =*/          "Polar Bear",
      /*.manufacturer =*/       "Daredevil_Discs",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17300,
      /*.radius =*/             0.10600,
      /*.rim_width =*/          0.01000,
      /*.thickness =*/          0.02200,
      /*.rim_depth =*/          0.01450,
      /*.rim_camber_height =*/  0.00700,
      /*.dome_height =*/        0.00350,
    },
    // 2
    Disc_Model
    { // Discmania DDX (S-Line)
      /*.mold_name =*/          "DDX",
      /*.manufacturer =*/       "Discmania",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02250,
      /*.thickness =*/          0.01550,
      /*.rim_depth =*/          0.01100,
      /*.rim_camber_height =*/  0.00350,
      /*.dome_height =*/        0.00300,
    },
    // 3
    Disc_Model
    { // Discraft Avenger SS (X)
      /*.mold_name =*/          "Avenger SS",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10525,
      /*.rim_width =*/          0.01900,
      /*.thickness =*/          0.01700,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00475,
      /*.dome_height =*/        0.00500,
    },
    // 4
    Disc_Model
    { // Discraft Buzzz Foil (Pro-D)
      /*.mold_name =*/          "Buzzz Foil",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Midrange",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.18000,
      /*.radius =*/             0.11000,
      /*.rim_width =*/          0.01300,
      /*.thickness =*/          0.01800,
      /*.rim_depth =*/          0.01300,
      /*.rim_camber_height =*/  0.00700,
      /*.dome_height =*/        0.00200,
    },
    // 5
    Disc_Model
    { // Discraft Challenger (Pro-D)
      /*.mold_name =*/          "Challenger",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17400,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.01000,
      /*.thickness =*/          0.02000,
      /*.rim_depth =*/          0.01500,
      /*.rim_camber_height =*/  0.00650,
      /*.dome_height =*/        0.00225,
    },
    // 6
    Disc_Model
    { // Discraft Crank (Z)
      /*.mold_name =*/          "Crank",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17400,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02350,
      /*.thickness =*/          0.01450,
      /*.rim_depth =*/          0.01125,
      /*.rim_camber_height =*/  0.00550,
      /*.dome_height =*/        0.00300,
    },
    // 7
    Disc_Model
    { // Discraft Fierce (Durable)
      /*.mold_name =*/          "Fierce",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17400,
      /*.radius =*/             0.10550,
      /*.rim_width =*/          0.01000,
      /*.thickness =*/          0.01900,
      /*.rim_depth =*/          0.01500,
      /*.rim_camber_height =*/  0.00600,
      /*.dome_height =*/        0.00275,
    },
    // 8
    Disc_Model
    { // Discraft Heat (Pro-D)
      /*.mold_name =*/          "Heat",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17200,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.01900,
      /*.thickness =*/          0.01800,
      /*.rim_depth =*/          0.01300,
      /*.rim_camber_height =*/  0.00400,
      /*.dome_height =*/        0.00550,
    },
    // 9
    Disc_Model
    { // Discraft Mini Buzzz (Jawbreaker)
      /*.mold_name =*/          "Mini Buzzz",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Midrange",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.06400,
      /*.radius =*/             0.07500,
      /*.rim_width =*/          0.00900,
      /*.thickness =*/          0.01225,
      /*.rim_depth =*/          0.00900,
      /*.rim_camber_height =*/  0.00500,
      /*.dome_height =*/        0.00050,
    },
    // 10
    Disc_Model
    { // Discraft Zone (Jawbreaker)
      /*.mold_name =*/          "Zone",
      /*.manufacturer =*/       "Discraft",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17400,
      /*.radius =*/             0.10700,
      /*.rim_width =*/          0.01150,
      /*.thickness =*/          0.01650,
      /*.rim_depth =*/          0.01400,
      /*.rim_camber_height =*/  0.00800,
      /*.dome_height =*/        0.00000,
    },
    // 11
    Disc_Model
    { // Infinite_Discs Exodus (I-Blend)
      /*.mold_name =*/          "Exodus",
      /*.manufacturer =*/       "Infinite_Discs",
      /*.disc_type =*/          "Fairway Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10550,
      /*.rim_width =*/          0.01800,
      /*.thickness =*/          0.01900,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00750,
      /*.dome_height =*/        0.00800,
    },
    // 12
    Disc_Model
    { // Innova Ape (B-Champion)
      /*.mold_name =*/          "Ape",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.16000,
      /*.radius =*/             0.10512,
      /*.rim_width =*/          0.02450,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00475,
      /*.dome_height =*/        0.00400,
    },
    // 13
    Disc_Model
    { // Innova Beast (Star)
      /*.mold_name =*/          "Beast",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02100,
      /*.thickness =*/          0.01550,
      /*.rim_depth =*/          0.01175,
      /*.rim_camber_height =*/  0.00625,
      /*.dome_height =*/        0.00400,
    },
    // 14
    Disc_Model
    { // Innova Boss (Champion)
      /*.mold_name =*/          "Boss",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10525,
      /*.rim_width =*/          0.02450,
      /*.thickness =*/          0.01650,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00600,
      /*.dome_height =*/        0.00200,
    },
    // 15
    Disc_Model
    { // Innova Colossus (G-Star)
      /*.mold_name =*/          "Colossus",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.16000,
      /*.radius =*/             0.10525,
      /*.rim_width =*/          0.02450,
      /*.thickness =*/          0.01550,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00500,
      /*.dome_height =*/        0.00325,
    },
    // 16
    Disc_Model
    { // Innova Destroyer (DX)
      /*.mold_name =*/          "Destroyer",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10450,
      /*.rim_width =*/          0.02450,
      /*.thickness =*/          0.01700,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00375,
      /*.dome_height =*/        0.00500,
    },
    // 17
    Disc_Model
    { // Innova Firebird (Champion)
      /*.mold_name =*/          "Firebird",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10600,
      /*.rim_width =*/          0.01950,
      /*.thickness =*/          0.01700,
      /*.rim_depth =*/          0.01175,
      /*.rim_camber_height =*/  0.00700,
      /*.dome_height =*/        0.00300,
    },
    // 18
    Disc_Model
    { // Innova Foxbat (Star)
      /*.mold_name =*/          "Foxbat",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Midrange",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17200,
      /*.radius =*/             0.10850,
      /*.rim_width =*/          0.01300,
      /*.thickness =*/          0.01850,
      /*.rim_depth =*/          0.01500,
      /*.rim_camber_height =*/  0.00825,
      /*.dome_height =*/        0.00450,
    },
    // 19
    Disc_Model
    { // Innova Katana (Star)
      /*.mold_name =*/          "Katana",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10450,
      /*.rim_width =*/          0.02400,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00400,
      /*.dome_height =*/        0.00350,
    },
    // 20
    Disc_Model
    { // Innova Krait (Champion)
      /*.mold_name =*/          "Krait",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02200,
      /*.thickness =*/          0.01850,
      /*.rim_depth =*/          0.01250,
      /*.rim_camber_height =*/  0.00550,
      /*.dome_height =*/        0.00450,
    },
    // 21
    Disc_Model
    { // Innova Leopard3 (G-Star)
      /*.mold_name =*/          "Leopard3",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Fairway Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10560,
      /*.rim_width =*/          0.01680,
      /*.thickness =*/          0.01510,
      /*.rim_depth =*/          0.01100,
      /*.rim_camber_height =*/  0.00700,
      /*.dome_height =*/        0.00250,
    },
    // 22
    Disc_Model
    { // Innova Mako3 (Champion)
      /*.mold_name =*/          "Mako3",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Midrange",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Convex",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10850,
      /*.rim_width =*/          0.01250,
      /*.thickness =*/          0.01700,
      /*.rim_depth =*/          0.01300,
      /*.rim_camber_height =*/  0.00800,
      /*.dome_height =*/        0.00075,
    },
    // 23
    Disc_Model
    { // Innova Monster (Champion)
      /*.mold_name =*/          "Monster",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02100,
      /*.thickness =*/          0.01450,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00575,
      /*.dome_height =*/        0.00275,
    },
    // 24
    Disc_Model
    { // Innova Shryke (Star)
      /*.mold_name =*/          "Shryke",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17100,
      /*.radius =*/             0.10575,
      /*.rim_width =*/          0.02350,
      /*.thickness =*/          0.01550,
      /*.rim_depth =*/          0.01100,
      /*.rim_camber_height =*/  0.00425,
      /*.dome_height =*/        0.00525,
    },
    // 25
    Disc_Model
    { // Innova Starfire (DX)
      /*.mold_name =*/          "Starfire",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.15000,
      /*.radius =*/             0.10450,
      /*.rim_width =*/          0.02100,
      /*.thickness =*/          0.01600,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00600,
      /*.dome_height =*/        0.00275,
    },
    // 26
    Disc_Model
    { // Innova Tomb (Star)
      /*.mold_name =*/          "Tomb",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Convex",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10600,
      /*.rim_width =*/          0.01050,
      /*.thickness =*/          0.01750,
      /*.rim_depth =*/          0.01400,
      /*.rim_camber_height =*/  0.00600,
      /*.dome_height =*/        0.00100,
    },
    // 27
    Disc_Model
    { // Innova Valkyrie (DX)
      /*.mold_name =*/          "Valkyrie",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10450,
      /*.rim_width =*/          0.01925,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00550,
      /*.dome_height =*/        0.00275,
    },
    // 28
    Disc_Model
    { // Innova Wraith (Star)
      /*.mold_name =*/          "Wraith",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.16700,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02200,
      /*.thickness =*/          0.01800,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00500,
      /*.dome_height =*/        0.00600,
    },
    // 29
    Disc_Model
    { // Innova Roc (DX)
      /*.mold_name =*/          "Roc",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "MidRange",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.15500,
      /*.radius =*/             0.10650,
      /*.rim_width =*/          0.01200,
      /*.thickness =*/          0.01950,
      /*.rim_depth =*/          0.01275,
      /*.rim_camber_height =*/  0.00700,
      /*.dome_height =*/        0.00300,
    },
    // 30
    Disc_Model
    { // Innova TeeBird3 (Champion)
      /*.mold_name =*/          "TeeBird3",
      /*.manufacturer =*/       "Innova",
      /*.disc_type =*/          "Fairway Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10600,
      /*.rim_width =*/          0.01900,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00600,
      /*.dome_height =*/        0.00400,
    },
    // 31
    Disc_Model
    { // Latitude_64 Pioneer (Opto)
      /*.mold_name =*/          "Pioneer",
      /*.manufacturer =*/       "Latitude_64",
      /*.disc_type =*/          "Fairway Driver",
      /*.stability =*/          "Overstable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17300,
      /*.radius =*/             0.10600,
      /*.rim_width =*/          0.01975,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00800,
      /*.dome_height =*/        0.00250,
    },
    // 32
    Disc_Model
    { // Loft Hydrogen (Alpha-Solid)
      /*.mold_name =*/          "Hydrogen",
      /*.manufacturer =*/       "Loft",
      /*.disc_type =*/          "Putter",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Flat",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10570,
      /*.rim_width =*/          0.00671,
      /*.thickness =*/          0.02100,
      /*.rim_depth =*/          0.01400,
      /*.rim_camber_height =*/  0.00870,
      /*.dome_height =*/        0.00100,
    },
    // 33
    Disc_Model
    { // MVP Wave (Mixed Premium)
      /*.mold_name =*/          "Wave",
      /*.manufacturer =*/       "MVP",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Understable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17500,
      /*.radius =*/             0.10550,
      /*.rim_width =*/          0.02100,
      /*.thickness =*/          0.01500,
      /*.rim_depth =*/          0.01200,
      /*.rim_camber_height =*/  0.00500,
      /*.dome_height =*/        0.00400,
    },
    // 34
    Disc_Model
    { // Prodigy_Discs D4 (400G)
      /*.mold_name =*/          "D4",
      /*.manufacturer =*/       "Prodigy_Discs",
      /*.disc_type =*/          "Distance Driver",
      /*.stability =*/          "Stable",
      /*.rim_camber_shape =*/   "Concave",
      /*.mass =*/               0.17000,
      /*.radius =*/             0.10500,
      /*.rim_width =*/          0.02400,
      /*.thickness =*/          0.01600,
      /*.rim_depth =*/          0.01150,
      /*.rim_camber_height =*/  0.00425,
      /*.dome_height =*/        0.00475,
    }
  };

}