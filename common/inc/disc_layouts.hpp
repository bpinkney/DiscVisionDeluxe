// Define disc layouts, and call-able inline look-up functions

#ifndef __DISC_LAYOUTS_HPP__
#define __DISC_LAYOUTS_HPP__

#include <map>
#include <string>

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

// Define a structure to hold the initial disc states measured by dvd_DvisEst
struct disc_init_state_t
{
  float lin_pos_xyz[3]; // m
  float lin_vel_xyz[3]; // m/s
  float ang_pos_hps[3]; // rad
  float ang_vel_hps[3]; // rad/s
  float wobble;         // [0,1] based on max_angle_delta / (PI * 0.5)
  DiscIndex discmold;   // defined in 'DiscIndex' enum above
};

// Disc definitions are statically defined so they can be looked up by all three projects
#define DISC_LAYOUTS_NUM  (16)
const disc_layout_t DISC_LAYOUTS[DISC_LAYOUTS_NUM] = 
{
  {NONE,        f36h11,   0,   0, 0},
  {GROUNDPLANE, f36h11, 386, 164, 0},
  {GROUNDPLANE_BIG, f36h11, 387, 489, 0},
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

// Define static getter/setter for key:value pair lookups used in dvd_DvisEst outputs
// this enum and hash function allow indirect use of 'switch/case' using strings
enum disc_init_state_string_code {
    einvalid,
    eposx,
    eposy,
    eposz,
    evelx,
    evely,
    evelz,
    ehyzer,
    epitch,
    espin,
    ehyzer_d,
    epitch_d,
    espin_d,
    ewobble,
    ediscmold
};

static inline disc_init_state_string_code disc_init_state_hash(std::string const& input_string) 
{
         if (input_string == "posx")      return eposx;
    else if (input_string == "posy")      return eposy;
    else if (input_string == "posz")      return eposz;
    else if (input_string == "velx")      return evelx;
    else if (input_string == "vely")      return evely;
    else if (input_string == "velz")      return evelz;
    else if (input_string == "hyzer")     return ehyzer;
    else if (input_string == "pitch")     return epitch;
    else if (input_string == "spin")      return espin;
    else if (input_string == "hyzer")     return ehyzer;
    else if (input_string == "pitch")     return epitch;
    else if (input_string == "spin")      return espin;
    else if (input_string == "hyzer_d")   return ehyzer_d;
    else if (input_string == "pitch_d")   return epitch_d;
    else if (input_string == "spin_d")    return espin_d;
    else if (input_string == "wobble")    return ewobble;
    else if (input_string == "discmold")  return ediscmold;
}
// always pass out to float (user can cast back to int if required, no large ints should be present here)
static inline bool dvd_DvisEst_get_disc_init_state_key_value(std::string key, disc_init_state_t * disc_init_state, float * value)
{
  bool got_value = false;
  switch(disc_init_state_hash(key))
  {
    case eposx:
      *value = (*disc_init_state).lin_pos_xyz[0];
      got_value = true;
      break;
    case eposy:
      *value = (*disc_init_state).lin_pos_xyz[1];
      got_value = true;
      break;
    case eposz:
      *value = (*disc_init_state).lin_pos_xyz[2];
      got_value = true;
      break;
    case evelx:
      *value = (*disc_init_state).lin_vel_xyz[0];
      got_value = true;
      break;
    case evely:
      *value = (*disc_init_state).lin_vel_xyz[1];
      got_value = true;
      break;
    case evelz:
      *value = (*disc_init_state).lin_vel_xyz[2];
      got_value = true;
      break;
    case ehyzer:
      *value = (*disc_init_state).ang_pos_hps[0];
      got_value = true;
      break;
    case epitch:
      *value = (*disc_init_state).ang_pos_hps[1];
      got_value = true;
      break;
    case espin:
      *value = (*disc_init_state).ang_pos_hps[2];
      got_value = true;
      break;
    case ehyzer_d:
      *value = (*disc_init_state).ang_vel_hps[0];
      got_value = true;
      break;
    case epitch_d:
      *value = (*disc_init_state).ang_vel_hps[1];
      got_value = true;
      break;
    case espin_d:
      *value = (*disc_init_state).ang_vel_hps[2];
      got_value = true;
      break;
    case ewobble:
      *value = (*disc_init_state).wobble;
      got_value = true;
      break;
    case ediscmold:
      *value = (float)(*disc_init_state).discmold;
      got_value = true;
      break;
  }
  return got_value;
}

static inline bool dvd_DvisEst_set_key_value(std::string key, float value, disc_init_state_t * disc_init_state)
{
  bool set_value = false;
  switch(disc_init_state_hash(key))
  {
    case eposx:
      (*disc_init_state).lin_pos_xyz[0] = value;
      set_value = true;
      break;
    case eposy:
      (*disc_init_state).lin_pos_xyz[1] = value;
      set_value = true;
      break;
    case eposz:
      (*disc_init_state).lin_pos_xyz[2] = value;
      set_value = true;
      break;
    case evelx:
      (*disc_init_state).lin_vel_xyz[0] = value;
      set_value = true;
      break;
    case evely:
      (*disc_init_state).lin_vel_xyz[1] = value;
      set_value = true;
      break;
    case evelz:
      (*disc_init_state).lin_vel_xyz[2] = value;
      set_value = true;
      break;
    case ehyzer:
      (*disc_init_state).ang_pos_hps[0] = value;
      set_value = true;
      break;
    case epitch:
      (*disc_init_state).ang_pos_hps[1] = value;
      set_value = true;
      break;
    case espin:
      (*disc_init_state).ang_pos_hps[2] = value;
      set_value = true;
      break;
    case ehyzer_d:
      (*disc_init_state).ang_vel_hps[0] = value;
      set_value = true;
      break;
    case epitch_d:
      (*disc_init_state).ang_vel_hps[1] = value;
      set_value = true;
      break;
    case espin_d:
      (*disc_init_state).ang_vel_hps[2] = value;
      set_value = true;
      break;
    case ewobble:
      (*disc_init_state).wobble = value;
      set_value = true;
      break;
    case ediscmold:
      (*disc_init_state).discmold = (DiscIndex)value;
      set_value = true;
      break;
  }
  return set_value;
}


#endif // __DISC_LAYOUTS_H__