#ifndef _CONFIG_H_
#define _CONFIG_H_

// this should be commented out if the code is to be compiled on windowz;
#define BUILD_WITH_GUI

// both btree and vnoc code-bases (these two come from two different
// code implementations) have now length unit in um
#define BTREE_TO_VNOC_SCALE_MULTIPLIER 1

// SHOW_DESTRUCTORS was used for low level debugging in hunting for
// a nasty bug; not used anymore;
//#define SHOW_DESTRUCTORS

#endif
