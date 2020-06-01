// Define disc layouts, and call-able inline look-up functions

#ifndef __DISC_LAYOUTS_HPP__
#define __DISC_LAYOUTS_HPP__

#include <map>

///--------------------------------------------------------------------------
/// @brief Disc Index Enum
enum DiscIndex
{
  NONE,
  GROUNDPLANE,
  GROUNDPLANE_BIG,
  PUTTER,
  PUTTER_OS,
  PUTTER_US,
  MIDRANGE,
  MIDRANGE_OS,
  MIDRANGE_US,
  FAIRWAY,
  FAIRWAY_OS,
  FAIRWAY_US,
  DRIVER,
  DRIVER_OS,
  DRIVER_US,
  SPECIAL  
};

///--------------------------------------------------------------------------
/// @brief AprilTag Families
enum TagFamily
{
  f16h5,
  f36h11,
  f41h12,
  f48h12,
  f52h13,
  f21h7,
  f49h12
};

///--------------------------------------------------------------------------
/// @brief Disc Interface/Virtual Structure
/// tag IDs can be used to look up properties for this
struct disc_layout_t
{
  DiscIndex     disc_index;
  TagFamily     tag_family;
  uint16_t      tag_id;  
  uint16_t      tag_size_mm;
  uint8_t       player;
};


// Disc definitions are statically defined so they can be looked up by all three projects
#define DISC_LAYOUTS_NUM  (16)
const disc_layout_t DISC_LAYOUTS[DISC_LAYOUTS_NUM] = 
{
  {NONE,        f36h11,   0,   0, 0},
  {GROUNDPLANE, f36h11, 386, 164, 0},
  {GROUNDPLANE_BIG, f36h11, 387, 164, 0},
  {PUTTER,      f36h11, 100, 109, 1},
  {PUTTER_OS,   f36h11, 101, 109, 1},
  {PUTTER_US,   f36h11, 102, 109, 1},
  {MIDRANGE,    f36h11, 103, 109, 1},
  {MIDRANGE_OS, f36h11, 104, 109, 1},
  {MIDRANGE_US, f36h11, 105, 109, 1},
  {FAIRWAY,     f36h11, 106, 109, 1},
  {FAIRWAY_OS,  f36h11, 107, 109, 1},
  {FAIRWAY_US,  f36h11, 108, 109, 1},
  {DRIVER,      f36h11, 109, 109, 1},
  {DRIVER_OS,   f36h11, 110, 109, 1},
  {DRIVER_US,   f36h11, 111, 109, 1},
  {SPECIAL,     f36h11, 112, 109, 1}
};

// Define a map so we can look up disc_layout from the detected AprilTag ID
const std::map<uint16_t, disc_layout_t> disc_layout_by_id
{ 
  {DISC_LAYOUTS[0].tag_id, DISC_LAYOUTS[0]},
  {DISC_LAYOUTS[1].tag_id, DISC_LAYOUTS[1]},
  {DISC_LAYOUTS[2].tag_id, DISC_LAYOUTS[2]},
  {DISC_LAYOUTS[3].tag_id, DISC_LAYOUTS[3]},
  {DISC_LAYOUTS[4].tag_id, DISC_LAYOUTS[4]},
  {DISC_LAYOUTS[5].tag_id, DISC_LAYOUTS[5]},
  {DISC_LAYOUTS[6].tag_id, DISC_LAYOUTS[6]},
  {DISC_LAYOUTS[7].tag_id, DISC_LAYOUTS[7]},
  {DISC_LAYOUTS[8].tag_id, DISC_LAYOUTS[8]},
  {DISC_LAYOUTS[9].tag_id, DISC_LAYOUTS[9]},
  {DISC_LAYOUTS[10].tag_id, DISC_LAYOUTS[10]},
  {DISC_LAYOUTS[11].tag_id, DISC_LAYOUTS[11]},
  {DISC_LAYOUTS[12].tag_id, DISC_LAYOUTS[12]},
  {DISC_LAYOUTS[13].tag_id, DISC_LAYOUTS[13]},
  {DISC_LAYOUTS[14].tag_id, DISC_LAYOUTS[14]},
  {DISC_LAYOUTS[15].tag_id, DISC_LAYOUTS[15]},
};



#endif // __DISC_LAYOUTS_H__