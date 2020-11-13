
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
      /*.mold_name =*/     "Mr Brick",
      /*.manufacturer =*/  "AAADiscs",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Stable",
      /*.mass =*/          3.0000,
      /*.radius =*/        0.1200,
      /*.rim_width =*/     0.0000,
      /*.thickness =*/     0.0700,
      /*.rim_depth =*/     0.0000,
      /*.edge_height =*/   0.0700
    },
    // 1
    Disc_Model
    { // Discmania P3 (C-Line)
      /*.mold_name =*/     "P3",
      /*.manufacturer =*/  "Discmania",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1054,
      /*.rim_width =*/     0.0126,
      /*.thickness =*/     0.0199,
      /*.rim_depth =*/     0.0131,
      /*.edge_height =*/   0.0093
    },
    // 2
    Disc_Model
    { // Discmania DDX (S-Line)
      /*.mold_name =*/     "DDX",
      /*.manufacturer =*/  "Discmania",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0225,
      /*.thickness =*/     0.0155,
      /*.rim_depth =*/     0.0110,
      /*.edge_height =*/   0.0070
    },
    // 3
    Disc_Model
    { // Discraft Buzzz (Pro-D)
      /*.mold_name =*/     "Buzzz",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Midrange",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1072,
      /*.rim_width =*/     0.0123,
      /*.thickness =*/     0.0159,
      /*.rim_depth =*/     0.0127,
      /*.edge_height =*/   0.0073
    },
    // 4
    Disc_Model
    { // Discraft Zombee (Z)
      /*.mold_name =*/     "Zombee",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Midrange",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1077,
      /*.rim_width =*/     0.0150,
      /*.thickness =*/     0.0162,
      /*.rim_depth =*/     0.0123,
      /*.edge_height =*/   0.0082
    },
    // 5
    Disc_Model
    { // Discraft Luna (Jawbreaker)
      /*.mold_name =*/     "Luna",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1051,
      /*.rim_width =*/     0.0108,
      /*.thickness =*/     0.0195,
      /*.rim_depth =*/     0.0143,
      /*.edge_height =*/   0.0074
    },
    // 6
    Disc_Model
    { // Discraft Soft Magnet (Pro-D)
      /*.mold_name =*/     "Soft Magnet",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1046,
      /*.rim_width =*/     0.0100,
      /*.thickness =*/     0.0202,
      /*.rim_depth =*/     0.0138,
      /*.edge_height =*/   0.0078
    },
    // 7
    Disc_Model
    { // Discraft Magnet (Jawbreaker)
      /*.mold_name =*/     "Magnet",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1730,
      /*.radius =*/        0.1055,
      /*.rim_width =*/     0.0090,
      /*.thickness =*/     0.0220,
      /*.rim_depth =*/     0.0140,
      /*.edge_height =*/   0.0140
    },
    // 8
    Disc_Model
    { // Discraft Zone (Jawbreaker)
      /*.mold_name =*/     "Zone",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1740,
      /*.radius =*/        0.1070,
      /*.rim_width =*/     0.0115,
      /*.thickness =*/     0.0165,
      /*.rim_depth =*/     0.0140,
      /*.edge_height =*/   0.0160
    },
    // 9
    Disc_Model
    { // Discraft Buzzz Foil (Pro-D)
      /*.mold_name =*/     "Buzzz Foil",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Midrange",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1800,
      /*.radius =*/        0.1100,
      /*.rim_width =*/     0.0130,
      /*.thickness =*/     0.0180,
      /*.rim_depth =*/     0.0130,
      /*.edge_height =*/   0.0110
    },
    // 10
    Disc_Model
    { // Discraft Heat (Pro-D)
      /*.mold_name =*/     "Heat",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1720,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0190,
      /*.thickness =*/     0.0180,
      /*.rim_depth =*/     0.0120,
      /*.edge_height =*/   0.0080
    },
    // 11
    Disc_Model
    { // Discraft Avenger SS (X)
      /*.mold_name =*/     "Avenger SS",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1052,
      /*.rim_width =*/     0.0190,
      /*.thickness =*/     0.0170,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0075
    },
    // 12
    Disc_Model
    { // Discraft Crank (Z)
      /*.mold_name =*/     "Crank",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1740,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0235,
      /*.thickness =*/     0.0145,
      /*.rim_depth =*/     0.0112,
      /*.edge_height =*/   0.0080
    },
    // 13
    Disc_Model
    { // Discraft Challenger (Pro-D)
      /*.mold_name =*/     "Challenger",
      /*.manufacturer =*/  "Discraft",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1740,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0100,
      /*.thickness =*/     0.0200,
      /*.rim_depth =*/     0.0150,
      /*.edge_height =*/   0.0163
    },
    // 14
    Disc_Model
    { // Infinite_Discs Exodus (I-Blend)
      /*.mold_name =*/     "Exodus",
      /*.manufacturer =*/  "Infinite_Discs",
      /*.disc_type =*/     "Fairway Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1055,
      /*.rim_width =*/     0.0180,
      /*.thickness =*/     0.0190,
      /*.rim_depth =*/     0.0120,
      /*.edge_height =*/   0.0100
    },
    // 15
    Disc_Model
    { // Innova TeeBird (Star)
      /*.mold_name =*/     "TeeBird",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Fairway Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1057,
      /*.rim_width =*/     0.0178,
      /*.thickness =*/     0.0160,
      /*.rim_depth =*/     0.0113,
      /*.edge_height =*/   0.0090
    },
    // 16
    Disc_Model
    { // Innova Leopard3 (G-Star)
      /*.mold_name =*/     "Leopard3",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Fairway Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1056,
      /*.rim_width =*/     0.0168,
      /*.thickness =*/     0.0151,
      /*.rim_depth =*/     0.0110,
      /*.edge_height =*/   0.0077
    },
    // 17
    Disc_Model
    { // Innova Shryke (Star)
      /*.mold_name =*/     "Shryke",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1710,
      /*.radius =*/        0.1057,
      /*.rim_width =*/     0.0235,
      /*.thickness =*/     0.0155,
      /*.rim_depth =*/     0.0110,
      /*.edge_height =*/   0.0080
    },
    // 18
    Disc_Model
    { // Innova Destroyer (DX)
      /*.mold_name =*/     "Destroyer",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1045,
      /*.rim_width =*/     0.0245,
      /*.thickness =*/     0.0170,
      /*.rim_depth =*/     0.0120,
      /*.edge_height =*/   0.0060
    },
    // 19
    Disc_Model
    { // Innova Krait (Champion)
      /*.mold_name =*/     "Krait",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0220,
      /*.thickness =*/     0.0185,
      /*.rim_depth =*/     0.0125,
      /*.edge_height =*/   0.0080
    },
    // 20
    Disc_Model
    { // Innova Wraith (Star)
      /*.mold_name =*/     "Wraith",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1670,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0220,
      /*.thickness =*/     0.0180,
      /*.rim_depth =*/     0.0120,
      /*.edge_height =*/   0.0090
    },
    // 21
    Disc_Model
    { // Innova Colossus (G-Star)
      /*.mold_name =*/     "Colossus",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1600,
      /*.radius =*/        0.1052,
      /*.rim_width =*/     0.0245,
      /*.thickness =*/     0.0155,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0090
    },
    // 22
    Disc_Model
    { // Innova Boss (Champion)
      /*.mold_name =*/     "Boss",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1052,
      /*.rim_width =*/     0.0245,
      /*.thickness =*/     0.0165,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0090
    },
    // 23
    Disc_Model
    { // Innova Mako3 (Champion)
      /*.mold_name =*/     "Mako3",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Midrange",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1085,
      /*.rim_width =*/     0.0125,
      /*.thickness =*/     0.0170,
      /*.rim_depth =*/     0.0130,
      /*.edge_height =*/   0.0145
    },
    // 24
    Disc_Model
    { // Innova Firebird (Champion)
      /*.mold_name =*/     "Firebird",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1060,
      /*.rim_width =*/     0.0195,
      /*.thickness =*/     0.0170,
      /*.rim_depth =*/     0.0118,
      /*.edge_height =*/   0.0120
    },
    // 25
    Disc_Model
    { // Innova Starfire (DX)
      /*.mold_name =*/     "Starfire",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1500,
      /*.radius =*/        0.1045,
      /*.rim_width =*/     0.0210,
      /*.thickness =*/     0.0160,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0095
    },
    // 26
    Disc_Model
    { // Innova Ape (B-Champion)
      /*.mold_name =*/     "Ape",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1600,
      /*.radius =*/        0.1051,
      /*.rim_width =*/     0.0245,
      /*.thickness =*/     0.0150,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0085
    },
    // 27
    Disc_Model
    { // Innova Katana (Star)
      /*.mold_name =*/     "Katana",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1045,
      /*.rim_width =*/     0.0240,
      /*.thickness =*/     0.0150,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0075
    },
    // 28
    Disc_Model
    { // Innova Monster (Champion)
      /*.mold_name =*/     "Monster",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0210,
      /*.thickness =*/     0.0145,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0105
    },
    // 29
    Disc_Model
    { // Innova Valkyrie (DX)
      /*.mold_name =*/     "Valkyrie",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1045,
      /*.rim_width =*/     0.0192,
      /*.thickness =*/     0.0150,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0080
    },
    // 30
    Disc_Model
    { // Innova Beast (Star)
      /*.mold_name =*/     "Beast",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Understable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0210,
      /*.thickness =*/     0.0155,
      /*.rim_depth =*/     0.0118,
      /*.edge_height =*/   0.0085
    },
    // 31
    Disc_Model
    { // Innova Tomb (Star)
      /*.mold_name =*/     "Tomb",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Putter",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1750,
      /*.radius =*/        0.1060,
      /*.rim_width =*/     0.0105,
      /*.thickness =*/     0.0175,
      /*.rim_depth =*/     0.0140,
      /*.edge_height =*/   0.0145
    },
    // 32
    Disc_Model
    { // Innova Foxbat (Star)
      /*.mold_name =*/     "Foxbat",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Midrange",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1720,
      /*.radius =*/        0.1085,
      /*.rim_width =*/     0.0130,
      /*.thickness =*/     0.0185,
      /*.rim_depth =*/     0.0150,
      /*.edge_height =*/   0.0130
    },
    // 33
    Disc_Model
    { // Latitude_64 Pioneer (Opto)
      /*.mold_name =*/     "Pioneer",
      /*.manufacturer =*/  "Latitude_64",
      /*.disc_type =*/     "Fairway Driver",
      /*.stability =*/     "Overstable",
      /*.mass =*/          0.1730,
      /*.radius =*/        0.1060,
      /*.rim_width =*/     0.0198,
      /*.thickness =*/     0.0150,
      /*.rim_depth =*/     0.0120,
      /*.edge_height =*/   0.0110
    },
    // 34
    Disc_Model
    { // Prodigy_Discs D4 (400G)
      /*.mold_name =*/     "D4",
      /*.manufacturer =*/  "Prodigy_Discs",
      /*.disc_type =*/     "Distance Driver",
      /*.stability =*/     "Stable",
      /*.mass =*/          0.1700,
      /*.radius =*/        0.1050,
      /*.rim_width =*/     0.0240,
      /*.thickness =*/     0.0160,
      /*.rim_depth =*/     0.0115,
      /*.edge_height =*/   0.0065
    }
  };

}