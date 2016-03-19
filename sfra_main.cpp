#include <config.h>
#include "sfra.h"
#include <stdio.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>

using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// launching point of the all the experiments;
//
////////////////////////////////////////////////////////////////////////////////

int main( int argc,char **argv) 
{
	// welcome;
	char welcome[] =
		"\n----------------------------------------------------------------------------\n"
		"sfra:    Simultaneous Floorplanning & Routers Assignment (Comp. "__DATE__")\n"
		"DA LAND: Design Automation Labs At North Dakota \n"
		"----------------------------------------------------------------------------\n";
	printf("%s", welcome);



	// (0) runtime;
	// cpu time: method 1;
	clock_t start_clock, end_clock;
	clock_t diff_clock;
	start_clock = clock();
	assert(start_clock != (clock_t)(-1));
	// cpu time: method 2;
	struct tms t1, t2;
	times(&t1);
	// wall clock;
	timeval start_wall, end_wall; // sec and microsec;
	gettimeofday( &start_wall, 0);



	// (1)
	// create the main object SFRA inside which all AFRA experiments will be run;
	SFRA sfra;
	sfra.parse_command_arguments(argc, argv); // gets a "seed" too;
	sfra.setup_seed(); // setup the seed for C++ RNG;
	sfra.print_setup_info(); // entertain user;



	// (A) in 2D and 25D simulation mode, things are simpler; this this 
	// the way 25D framework was studied for ReConFig'09 paper;
	if ( sfra.sim_mode() == SIMULATED_ARCH_25D ||
		 sfra.sim_mode() == SIMULATED_ARCH_2D) {

		// (2)
		//#########################################
		//# Try a 'n_fps' number of floorplanings #
		//#########################################

		// try 'n_fps' number of floorplanings; I should have "best_fps" as a 
		// sketch array inside SFRA;

		vector <ONE_OF_THE_BEST_FLOOR_PLANS> best_fps;
		sfra.search_n_fps_floorplans( best_fps);

		// (3)
		//###########################################################
		//# Read the list of best FP's, assign routers and simulate #
		//###########################################################

		// next we simulate using vNOC all floorplans from the bests list;
		// the result of each simulation is stored inside each fp object
		// from the list;

		printf("\nThe %d best floorplans will be simulated.\n\n", best_fps.size());
		sfra.routers_assignment_and_vNOC_simulation( best_fps);

		// (4)
		//###################
		//# Results summary #
		//###################

		// compute final results statistics and save in output files;

		sfra.calculate_final_results_statistics( argc, argv, best_fps);
	}
	// (B) in 3D I partition the task graph into 2 using hMetis and 
	// floorplan the partitions on layers 1 and 3; layer 2 is used
	// for the homogeneous network;
	else if ( sfra.sim_mode() == SIMULATED_ARCH_3D) {
	
		// (2)
		// -- create top level application graph
		// -- partition it into two
		// -- create sub-floorplans and call floorplanner; do it for n_fps
		//    and retain n_best;
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> best_fps1;
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> best_fps2;
		//sfra.search_n_fps_floorplans_3D( best_fps1, best_fps2);
		sfra.search_n_fps_floorplans_3D_version2( best_fps1, best_fps2);
		
		// (3) assign routers to the IP/core from both subfloorplans;
		// simulated using vNOC;
		printf("\nThe %d best floorplans will be simulated.\n\n", best_fps1.size());
		sfra.routers_assignment_and_vNOC_simulation_3D( best_fps1, best_fps2);
		
		// (4) compute final results statistics and save in output files;

		sfra.calculate_final_results_statistics( argc, argv, best_fps1);
	}



	// (5) runtime;
	// cpu time;
	end_clock = clock();
	assert(end_clock != (clock_t)(-1));
	diff_clock = end_clock - start_clock;
	printf("vnoc3 total cputime (processor time, clock()) = %.3f sec\n",
		(double)diff_clock/CLOCKS_PER_SEC);
	// cpu time: method 2;
	times(&t2);
	printf("vnoc3 total cputime (processor time, tms) = %.3f sec\n",
		(double)(t2.tms_utime-t1.tms_utime)/HZ);
	// wall clock;
	gettimeofday( &end_wall, 0);
	double diff_sec_usec = end_wall.tv_sec - start_wall.tv_sec + 
		double(end_wall.tv_usec - start_wall.tv_usec) / 1000000.0;
	printf ("vnoc3 total walltime (wall clock time) = %.3f sec\n", diff_sec_usec);
	printf("\n");
}
