#ifndef _VNOC_APP_
#define _VNOC_APP_
#include "config.h"
#include "sfra.h"
#include "vnoc_app.h"
#include "vnoc_topology.h"
#include "vnoc_event.h"
#include "vnoc.h"
#include <time.h>

#ifdef BUILD_WITH_GUI
#include "sfra_gui.h"
#endif

using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// VNOC_APPLICATION
//
////////////////////////////////////////////////////////////////////////////////

VNOC_APPLICATION::VNOC_APPLICATION( SFRA *sfra, // its owner;
									ROUTER_ASSIGNMENT *router_assignment,
									long net_size, long inp_buf, long out_buf, long vc_n,
									long flit_size, long link_bw, double link_l,
									long pipeline_in_link, string trace_file, long seed,
									ROUTING_ALGORITHM routing_a, double cycles,
									double warmup_cycles, bool use_gui,
									ROUTERS_DISTRIBUTION *r_distrib_p,
									int gui_pauses,
									ONE_OF_THE_BEST_FLOOR_PLANS *best_fp_p1,
									ONE_OF_THE_BEST_FLOOR_PLANS *best_fp_p2)
{
	// this wrapper contains here basically what the vnoc code had in its
	// main function; with additional floorplanning related args;
	_sfra = sfra;

	time_t start_time, end_time;
	clock_t start_clock, end_clock;
	clock_t diff_clock;
	time_t diff_time;
	start_time = time(NULL);
	start_clock = clock();
	assert(start_time != (time_t)(-1));
	assert(start_clock != (clock_t)(-1));

	// read-in the topology file and sanity checks;
	TOPOLOGY topology( net_size, inp_buf, out_buf, vc_n, flit_size, link_bw,link_l,
					   pipeline_in_link, trace_file, seed, routing_a, cycles,
					   warmup_cycles, use_gui);

	EVENT_QUEUE event_queue( 0.0, &topology); // start time = 0.0;
	VNOC vnoc(&topology, &event_queue, router_assignment, sfra->verbose()); // create network;
	// we save a pointer to sfra host object because inside vNOC we use
	// some Elmore delay calculations, etc. that we need the retrieve from
	// the host platform from where we call the vNOC simulator thru its
	// wrapper;
	vnoc.set_sfra_host( sfra);
	event_queue.set_vnoc(&vnoc);

	// link_l is now known for this RxR reguar mesh; it will determine
	// T clock for the whole NoC;
	// Note: square_proportionality is set to be false; that is we assume
	// wires to be buffered that makes the delay linearly dependent 
	// of link length;
	vnoc.compute_heterogeneous_t_clock(
		link_l,
		false); // true = square, false = linear proportionality;



	// gui stuff: in effect if on Linux and not if on Windowz;
	#ifdef BUILD_WITH_GUI
	GUI_GRAPHICS gui( &topology, &vnoc,
		_sfra->test_name(), // testcase name, displayed in postscript screenshots;
		best_fp_p1,
		best_fp_p2, // used only by 3D;
		router_assignment, r_distrib_p);
	vnoc.set_gui(&gui); // initially gui is empty;

	if ( topology.use_gui()) { // if user asked to use the gui;
		// mark flag that we are gonna use the gui; set is_gui_usable
		// and wait_for_user_input_automode; then build;
		gui.set_graphics_state( true, gui_pauses);
		gui.build();
		gui.init_draw_coords( 100.0, 0.0);
	} else { // gui is not usable;
		gui.set_graphics_state( false, 1);
	}
	#endif


	vnoc.run_simulation();
	vnoc.print_simulation_results();

	end_time = time(NULL);
	end_clock = clock();
	assert(end_time != (time_t)(-1));
	assert(end_clock != (clock_t)(-1));
	diff_time = end_time - start_time;
	diff_clock = end_clock - start_clock;
	printf("\n");
	// printf ("cputime : start_clock = %lu units, end_clock = %lu units\n", start_clock, end_clock);
	printf("cputime : processor time used = %.3f sec\n", (double)diff_clock/CLOCKS_PER_SEC);
	// printf("walltime : start_time = %lu sec, end_time = %lu sec\n", start_time, end_time);
	// printf("walltime : elapsed (wall clock) time = %lu sec\n", diff_time);
	printf("\n");


	#ifdef BUILD_WITH_GUI
	if ( gui.is_gui_usable()) {
		gui.close_graphics(); // close down X Display
	}
	#endif
}

#endif
