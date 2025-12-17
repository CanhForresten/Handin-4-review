#pragma once
#include <cstdint>

/* Display is addressed as follows:
 *  (0,0)..........................(0,127)
 *       ..........................
 *       ..........................
 *       ..........................
 * (31,0)..........................(31,127)
 */

// 8x8 ball bitmap
const uint8_t ballBitmap[] = {
    0x3C, // ..####..
    0x7E, // .######.
    0xDB, // ##.##.##
    0xFF, // ########
    0xFF, // ########
    0xDB, // ##.##.##
    0x7E, // .######.
    0x3C  // ..####..
};

const int ballHeight = 8;
const int ballWidth = 8;