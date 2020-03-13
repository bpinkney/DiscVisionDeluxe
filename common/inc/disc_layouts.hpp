// Define disc layouts, and call-able inline look-up functions

#ifndef __DISC_LAYOUTS_HPP__
#define __DISC_LAYOUTS_HPP__

#include <map>

///--------------------------------------------------------------------------
/// @brief Disc Index Enum
enum DiscIndex
{
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
#define DISC_LAYOUTS_NUM  (13)
const disc_layout_t DISC_LAYOUTS[DISC_LAYOUTS_NUM] = 
{
  {PUTTER,      f36h11, 100, 112, 1},
  {PUTTER_OS,   f36h11, 101, 112, 1},
  {PUTTER_US,   f36h11, 102, 112, 1},
  {MIDRANGE,    f36h11, 103, 112, 1},
  {MIDRANGE_OS, f36h11, 104, 112, 1},
  {MIDRANGE_US, f36h11, 105, 112, 1},
  {FAIRWAY,     f36h11, 106, 112, 1},
  {FAIRWAY_OS,  f36h11, 107, 112, 1},
  {FAIRWAY_US,  f36h11, 108, 112, 1},
  {DRIVER,      f36h11, 109, 112, 1},
  {DRIVER_OS,   f36h11, 110, 112, 1},
  {DRIVER_US,   f36h11, 111, 112, 1},
  {SPECIAL,     f36h11, 112, 112, 1}
};

// Define a map so we can look up disc_layout from the detected AprilTag ID
const std::map<uint16_t, disc_layout_t> disc_layout_by_id
{ 
  {DISC_LAYOUTS[0].tad_id, DISC_LAYOUTS[0]}, 
};



#endif // __DISC_LAYOUTS_H__