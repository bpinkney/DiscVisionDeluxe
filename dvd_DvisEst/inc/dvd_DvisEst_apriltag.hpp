#ifndef DVD_DVISEST_APRILTAG_HPP
#define DVD_DVISEST_APRILTAG_HPP

// Scout thread looks N frames from the back of the image queue, and discards old frames if not apriltags are detected (real time)
// Meas thread starts from the front of the queue, and processes all entries (results in not real time)
#define AT_DETECTION_THREAD_MODE_SCOUT (0)
#define AT_DETECTION_THREAD_MODE_MEAS  (1)

// should this match the meas queue count in the estimate meas queue? probably not
// in general, it seems like this should be << than the meas queue to account for
// the variability in apriltag detection speed
#define AT_THREAD_COUNT     (6)
#define AT_INT_THREAD_COUNT (2)

bool dvd_DvisEst_apriltag_test(void);
// Get centroid of apriltag pixel intensities
float dvd_DvisEst_apriltag_get_at_pixel_centroid(void);
// Init apriltag threads, etc.
void dvd_DvisEst_apriltag_init(const bool convert_from_bayer, const bool calc_groundplane);
// Join all apriltag threads
void dvd_DvisEst_apriltag_end(void);

#endif // DVD_DVISEST_APRILTAG_HPP
