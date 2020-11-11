
#pragma once
namespace DfisX
{



  // For now, this is just a hard-coded mapping to fixed discs from test data
  // and using the ordering of the 'DiscIndex' enums
  // duplicate entries are just copied from the destroyer stats

  /*//Disc Model
  //contains the name and the physics/aerodynamic properties of a disc mold
  struct Disc_Model
  {
    std::string mold_name;
    float edge_height;   // approximation of 'edge height' for our simpified disc model (m)
    float cavity_depth;  // height of cavity, measured at the edge of the cavity (m)
    float rim_width;     // width of rim, from edge of cavity, to edge of disc (m)
    float camber_height; // max height of camber above edge (m)
    
    float mass;
    float diameter;
    float radius;
    float area;
  };*/

  const std::vector<DfisX::Disc_Model> disc_object_array = 
  {
    // 0
    Disc_Model
    {  // make the 'NONE' one a brick
      .mold_name = "NONE", 
      .edge_height = 0.1, 
      .cavity_depth = 0.0, 
      .rim_width = 0.0, 
      .camber_height = 0.0, 
      .mass = 3.0,
      .radius = 0.13
    },
    // 1
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST1", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 2
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST2", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 3
    Disc_Model
    {
      .mold_name = "MAGNET_JAWBREAKER", 
      .edge_height = 0.013, 
      .cavity_depth = 0.014, 
      .rim_width = 0.009, 
      .camber_height = (0.022-0.013), //thickness - edge height
      .mass = 0.173,
      .radius = (0.211 * 0.5)
    },
    // 4
    Disc_Model
    {
      .mold_name = "ZONE_JAWBREAKER", 
      .edge_height = 0.0115, 
      .cavity_depth = 0.014, 
      .rim_width = 0.0115,
      .camber_height = (0.0165-0.0115), //thickness - edge height
      .mass = 0.174,
      .radius = (0.214 * 0.5)
    },
    // 5
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST5", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 6
    Disc_Model
    {
      .mold_name = "BUZZZ_BIG_Z", 
      .edge_height = 0.011, 
      .cavity_depth = 0.013, 
      .rim_width = 0.013, 
      .camber_height = (0.018-0.011), //thickness - edge height
      .mass = 0.180,
      .radius = (0.220 * 0.5)
    },
    // 7
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST7", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 8
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST8", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 9
    Disc_Model
    {
      .mold_name = "TBIRD_STAR", 
      .edge_height = 0.009, 
      .cavity_depth = 0.0113, 
      .rim_width = 0.0178, 
      .camber_height = (0.016-0.009), //thickness - edge height
      .mass = 0.175,
      .radius = (0.2114 * 0.5)
    },
    // 10
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST10", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 11
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST11", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 12
    Disc_Model
    {
      .mold_name = "SHRYKE_STAR", 
      .edge_height = 0.008, 
      .cavity_depth = 0.011, 
      .rim_width = 0.0235, 
      .camber_height = (0.0155-0.008), //thickness - edge height
      .mass = 0.171,
      .radius = (0.2115 * 0.5)
    },
    // 13
    Disc_Model
    {
      .mold_name = "DESTROYER_DX", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 14
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST14", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    },
    // 15
    Disc_Model
    {  // copied from a destroyer for now, we can use this to test new configs for now
      .mold_name = "TEST15", 
      .edge_height = 0.006, 
      .cavity_depth = 0.012, 
      .rim_width = 0.0245, 
      .camber_height = (0.017-0.006), //thickness - edge height
      .mass = 0.170,
      .radius = (0.209 * 0.5)
    }
  };

}