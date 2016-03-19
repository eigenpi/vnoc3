#ifndef _VNOC_APPLICATION_H_
#define _VNOC_APPLICATION_H_

#include "config.h"
#include "vnoc_topology.h"
#include "vnoc.h"
#include <string>

using namespace std;

class ROUTER_ASSIGNMENT;

////////////////////////////////////////////////////////////////////////////////
//
// VNOC_APPLICATION
// 
// this is basically a wrapper class that calls the vnoc simulator for
// a floorplan instance; it creates a new vnoc object every time when its
// ctor is called; we should think of creating a vnoc object once and then 
// re-use it as many times as needed; that way we save runtime and less 
// memory fragmentation;
//
////////////////////////////////////////////////////////////////////////////////

class VNOC_APPLICATION {
 public:
	SFRA *_sfra;

 public:
	VNOC_APPLICATION( SFRA *sfra, // its owner;
		ROUTER_ASSIGNMENT *router_assignment,
		long net_size, long inp_buf, long out_buf, long vc_n,
		long flit_size, long link_bw, double link_l,
		long pipeline_in_link, string trace_file, long seed,
		ROUTING_ALGORITHM routing_a, double cycles,
		double warmup_cycles, bool use_gui,
		ROUTERS_DISTRIBUTION *r_distrib_p,
		int gui_pauses,
		ONE_OF_THE_BEST_FLOOR_PLANS *best_fp_p1,
		ONE_OF_THE_BEST_FLOOR_PLANS *best_fp_p2 = 0); // used only by 3D;
	~VNOC_APPLICATION() {}

	SFRA *sfra() { return _sfra; }
};

#endif
