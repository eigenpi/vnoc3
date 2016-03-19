#include "config.h"
#include <utility>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <climits>
#include <cfloat>
#include "vnoc_topology.h"
#include "sfra.h"
#include "sfra_hungarian.h"
#include "vnoc_app.h"
#include "fp_btree.h"
#include "fp_sa.h"

using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// ROUTERS_DISTRIBUTION
//
////////////////////////////////////////////////////////////////////////////////

ROUTERS_DISTRIBUTION::ROUTERS_DISTRIBUTION(long x, long y, long x0, long y0, double ll) 
{
	_nx = x; 
	_ny = y;
	_sx = x0; 
	_sy = y0;
	_link_length = ll;
}

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_ASSIGNMENT
//
////////////////////////////////////////////////////////////////////////////////

ROUTER_ASSIGNMENT::ROUTER_ASSIGNMENT( int ary) 
{
	_n_rows = ary; 
	_n_columns = ary;
	_array_ids = (int**) malloc(_n_rows * sizeof(int *));
	_array_corners = (CORNER_NAME**) malloc(_n_rows * sizeof(int *));
	_array_extralinks = (double**) malloc(_n_rows * sizeof(double *));
	for ( int i = 0; i < _n_rows; i++) {
		_array_ids[i] = (int *) malloc(_n_columns * sizeof(int));
		_array_corners[i] = (CORNER_NAME *) malloc(_n_columns * sizeof(int));
		_array_extralinks[i] = (double *) malloc(_n_columns * sizeof(double));
	}
	clear_assignments();
}

void ROUTER_ASSIGNMENT::assign(int x,int y, int id, 
	CORNER_NAME corner, double extralink) 
{
	_array_ids[x][y] = id;
	_array_corners[x][y] = corner;
	_array_extralinks[x][y] = extralink;
	_total_extralinks += extralink;
}

int ROUTER_ASSIGNMENT::assigned(int x, int y)
{
	if ( x >= _n_columns || x < 0 || y >= _n_rows || y < 0) {
		printf("ERROR: trying to verify assignment of router %d.%d->%d (_n_columns=%d,_n_rows=%d)\n",
			x, y, _array_ids[x][y], _n_columns, _n_rows);
		exit(1);
	}
	return _array_ids[x][y];
}

CORNER_NAME ROUTER_ASSIGNMENT::corner_used(int x, int y) 
{
	if ( x >= _n_columns || x < 0 || y >= _n_rows || y < 0) {
		printf("ERROR: trying to verify corner used by router %d.%d (_n_columns=%d,_n_rows=%d)\n",
			x, y, _n_columns, _n_rows);
		exit(1);
	}
	return _array_corners[x][y];
}

pair<int, int> ROUTER_ASSIGNMENT::core_id_to_router_xy(int id) 
{
	int found_x = -1;
	int found_y = -1;
	pair<int, int> temp_xy;

	for ( int j = 0; j < _n_rows; j++) {
		for( int i = 0; i < _n_columns; i++) {
			if ( _array_ids[i][j] == id) {
				found_x = i; 
				found_y = j;
			}
		}
	}
	assert( found_x != -1 && found_y != -1);
	temp_xy.first = found_x; 
	temp_xy.second = found_y;
	return temp_xy;
}

CORNER_NAME ROUTER_ASSIGNMENT::corner_used_by_id(int id) 
{
	int found_x = -1;
	int found_y = -1;
	CORNER_NAME temp_corner;

	for ( int j = 0; j < _n_rows; j++) {
		for(int i = 0; i < _n_columns; i++) {
			if ( _array_ids[i][j] == id) {
				found_x = i;
				found_y = j;
			}
		}
	}
	assert( found_x != -1 && found_y != -1);
	temp_corner = _array_corners[found_x][found_y];
	return temp_corner;
}

double ROUTER_ASSIGNMENT::extralink_by_id(int id)
{
	int found_x = -1;
	int found_y = -1;

	for ( int j = 0; j < _n_rows; j++) {
		for(int i = 0;i < _n_columns; i++) {
			if ( _array_ids[i][j] == id){
				found_x = i; 
				found_y = j;
			}
		}
	}
	assert(found_x != -1 && found_y != -1);
	return _array_extralinks[found_x][found_y];
}

void ROUTER_ASSIGNMENT::clear_assignments()
{
	// clear all assignments;
	for ( int y = 0; y < _n_rows; y++) {
		for ( int x = 0; x < _n_columns; x++) {
			_array_ids[x][y] = -1;
		}
	}
	_total_extralinks = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
//
// SFRA - means simultaneous floorplanning and router assignment;
//
////////////////////////////////////////////////////////////////////////////////

bool SFRA::parse_command_arguments( int argc, char *argv[]) 
{
	bool result = true;
	int i = 0;

	// topology and trace files are mandatory;
	if (argc < 3 || argv[1]=="--help" || argv[1]=="-h" || argv[1]=="-?" || argv[1]=="/?") {
		printf("\nUsage: sfra file: (test file) cycles: (int) warmup: (int) [Options...]\n\n");
		printf(" Option\t\tDescription - default internal value\n");
		printf("SIMULATED ANNEALING FLOORPLANNING:\n");
		printf(" [times:]\tUsed to multiply num of cores to obtain num of permutations in one temperature - 160\n");
		printf(" [local:]\tNumber of local search iterations - 6\n");
		printf(" [term_temp:]\tAnnealing final/terminating temperature (starts at 1) - 0.1\n");
		printf(" [avg_ratio:]\tUsed in annealing cost calculations - 30\n");
		printf(" [lambda:]\tUsed in the exponential for probability calculation - 1.3\n");
		printf(" [alpha:]\tAnnealing cost ratio (from 0-wirelength to 1-area) - 0.25\n");
		printf(" [seed:]\tRandom number generator seed - use internal clock\n");
		printf(" [scale:]\tScale multiplier - disabled by default\n");
		printf("VNOC SIMULATION:\n");
		printf(" [inp_buf:]\tRouters input buffers size (in flits) - 12\n");
		printf(" [out_buf:]\tRouters output buffers size (in flits) - 12\n");
		printf(" [vc_n:]\tNumber of virtual channels - 3\n");
		printf(" [flit_size:]\tFlit size - 4\n");
		printf(" [link_bw:]\tLink bandwidth - 64\n");
		//printf(" [pipe_link:]\tNumber of pipeline stages per link - 0\n");
		printf(" [routing_a:]\tRouting algorithm. Must be XY or TXY - XY\n");
		printf(" [extra:]\tExtra-links timing factor. Use 0 to disable them - 1\n");
		printf("FRAMEWORK'S TOP-LEVEL LOOP:\n");
		printf(" [mode:]\tArchitecture selection, 2D, 3D or 2.5D - 2.5D\n");
		printf(" [n_fps:]\tNumber of floorplans to try (attempts) - 10\n");
		printf(" [n_best:]\tNumber of best floorplans to be simulated - 3\n");
		printf(" [x_ary:]\tExtra routers per dimension - 0\n");
		printf(" [fp_criteria:]\tCriteria for \"best\" fp's (W-wirelengh, A-area) - A\n");
		printf(" [gui]\t\tUse the graphical user interface (GUI) - disabled by default\n");
		printf(" [p:]\t\tGUI pausing behavior per simulation. 0:step-by-step, 1:once at\n\t\tthe end, 2:never pauses - 1\n");
		printf(" [verbose:]\tPrint debugging info, 0 or 1 - 1\n");
		printf(" [name:]\tOutputs the average latency of all simulations to a file\n\t\tnamed results\\results.txt with a \"simulation name\" tag -\n\t\tdisabled by default\n");
		printf(" [load:]\tAdjust the injection load by 10,20,30...100 percent (use only\n\t\tmultiples of 10) - 100\n");
		printf(" [loadsweep]\tSweeps the \"injection-load\" from 10-100%% - disabled\n");
		printf(" [bufsweep]\tSweeps buffers-size from 1x to 5x - disabled\n");
		printf(" [excel]\tIf load or buffer-size sweep is used, outputs an excel file - disabled\n\n");
		exit(1);
	}

	i = 1;
	while ( i < argc) {
		if ( !strcmp(argv[i], "file:")) {
			strcpy(_inputfile, argv[i+1]);
			i += 2; // skip the next parameter;
			continue;
		}
		if ( !strcmp(argv[i], "times:")) {
			_times = atoi(argv[i+1]);
			if (_times < 1 || _times > LONG_MAX) { 
				printf("Error:\ttimes value must be between [1 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "local:")) {
			_local = atoi(argv[i+1]);
			if (_local < 0 || _local > LONG_MAX) { 
				printf("Error:\tlocal value must be between [0 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "avg_ratio:")) {
			_avg_ratio = atoi(argv[i+1]);
			if (_avg_ratio < 1 || _avg_ratio > LONG_MAX) { 
				printf("Error:\tavg_ratio value must be between [1 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "lambda:")) {
			_lambda = atof(argv[i+1]);
			if (_lambda < 0 || _lambda > 10) { 
				printf("Error:\tlambda value must be between [0 10].\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "term_temp:")) {
			_term_temp = atof(argv[i+1]);
			if (_term_temp < 0 || _term_temp > 1) { 
				printf("Error:\tterm_temp value must be between [0 1].\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp( argv[i], "alpha:")) {
			_alpha = atof(argv[i+1]);
			if (_alpha < 0 || _alpha > 1) { 
				printf("Error:\talpha value must be between [0 1].\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "scale:")) {
			_fp_scale = atof(argv[i+1]);
			if (_fp_scale < 0.0005 || _fp_scale > 5000) { 
				printf("Error:\tscale value must be between [0.0005 5000].\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "seed:")) {
			_seed = atoi(argv[i+1]);
			if (_seed < 1 || _seed > LONG_MAX) { 
				printf("Error:\tseed value must be between [1 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "n_fps:")) {
			_n_fps = atoi(argv[i+1]);
			if (_n_fps < 1 || _n_fps > LONG_MAX) { 
				printf("Error:\tn_fps value must be between [1 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "n_best:")) {
			_n_best = atoi(argv[i+1]);
			if (_n_best < 1 || _n_best > LONG_MAX) { 
				printf("Error:\tn_best value must be between [1 %ld].\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "fp_criteria:")) {
			_fp_criteria = argv[i+1][0];
			if (_fp_criteria != 'W' && _fp_criteria != 'A') { 
				printf("Error:\tfp_criteria must be between W or A.\n", LONG_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "x_ary:")) {
			_x_ary = atoi(argv[i+1]);
			if (_x_ary < 0 || _x_ary > INT_MAX) { 
				printf("Error:\tx_ary value must be between [0 %d].\n", INT_MAX);
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "verbose:")) {
			int v2i = atoi(argv[i+1]);
			if (v2i != 0 && v2i != 1) { 
				printf("Error:\tverbose value must be 0 or 1.\n");
				exit(1); 
			}
			if (v2i == 0) _verbose = false;
			else if (v2i == 1) _verbose = true;
			i += 2; 
			continue;
		}
		if ( !strcmp( argv[i], "mode:")) {
			if (strcmp(argv[i+1],"2D") && strcmp(argv[i+1],"3D") && 
				strcmp(argv[i+1],"2.5D")) { 
				printf("Error:\tmode must be 2D, 3D or 2.5D.\n", LONG_MAX);
				exit(1); 
			}
			if ( !strcmp(argv[i+1], "2D")){
				_sim_mode = SIMULATED_ARCH_2D;
			} else if ( !strcmp(argv[i+1], "3D")){
				_sim_mode = SIMULATED_ARCH_3D;
			} else {
				_sim_mode = SIMULATED_ARCH_25D;
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "inp_buf:")) {
			_inp_buf = atoi(argv[i+1]); 
			if (_inp_buf < 1 || _inp_buf > 1024) { 
				printf("Error:\tinp_buf value must be between [1 1024].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "out_buf:")) {
			_out_buf = atoi(argv[i+1]);
			if (_out_buf < 1 || _out_buf > 1024) { 
				printf("Error:\tout_buf value must be between [1 1024].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "vc_n:")) {
			_vc_n = atoi(argv[i+1]); 
			if (_vc_n < 1 || _vc_n > 128) { 
				printf("Error:\tvc_n value must be between [1 128].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "flit_size:")) {
			_flit_size = atoi(argv[i+1]);
			if (_flit_size < 1 || _flit_size > 128) { 
				printf("Error:\tflit_size value must be between [1 128].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "link_bw:")) {
			_link_bw = atoi(argv[i+1]);
			if (_link_bw < 1 || _link_bw > 128) { 
				printf("Error:\tlink_bw value must be between [1 128].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "pipe_link:")) {
			_pipeline_in_link = atoi(argv[i+1]);
			if (_pipeline_in_link < 0 || _pipeline_in_link > 32) { 
				printf("Error:\tpipe_link value must be between [0 32].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "routing_a:")) {
			if (argc <= i+1) {
				printf ("Error:\trouting_a option requires a string parameter.\n");
				exit (1);
			} 
			if (strcmp(argv[i+1], "XY") == 0) {
				_routing_a = ROUTING_XY;
			} 
			else if (strcmp(argv[i+1], "TXY") == 0) {
				_routing_a = ROUTING_TXY;
			} else {
				printf("Error:\trouting_a must be XY or TXY.\n");
				exit (1);
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "cycles:")) {
			_cycles = atoi(argv[i+1]);
			if (_cycles <= 0 || _cycles > 500000) { 
				printf("Error:\tcycles value must be between [1 500000].\n");
				exit(1); 
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "warmup:")) {
			_warmup_cycles = atoi(argv[i+1]);
			if (_warmup_cycles <= 0 || _warmup_cycles > 500000) { 
				printf("Error:\twarmup value must be between [1 500000].\n");
				exit(1);
			}
			i += 2; 
			continue;
		}
		if ( !strcmp(argv[i], "extra:")) {
			_extra_links_timing_factor = atof(argv[i+1]);
			if (_extra_links_timing_factor < 0 || 
				_extra_links_timing_factor > 10) { 
				printf("Error:\textra value must be between [0 10].\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "gui")) {
			_use_gui = true;
			i++; 
			continue;
		}
		if ( !strcmp(argv[i], "p:")) {
			_gui_pauses = atoi(argv[i+1]);
			if (_gui_pauses < 0 || _gui_pauses > 2) { 
				printf("Error:\tp value must be between 0, 1 or 2.\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "name:")) {
			strcpy(_test_name, argv[i+1]);
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "load:")) {
			_inj_load = atoi(argv[i+1]);
			if (_inj_load < 10 || _inj_load > 100 ||
				( ceil( _inj_load/10) != _inj_load/10) ) { 
				printf("Error:\tload value must be between [10 100] in steps of 10.\n");
				exit(1); 
			}
			i += 2;
			continue;
		}
		if ( !strcmp(argv[i], "loadsweep")) {
			_load_sweep = true;
			i++; 
			continue;
		}
		if ( !strcmp(argv[i], "bufsweep")) {
			_buffer_sweep = true;
			i++; 
			continue;
		}
		if ( !strcmp(argv[i], "excel")) {
			_use_excel = true;
			i++; 
			continue;
		}
		if ( !strcmp(argv[i], "TESTCASE:")) {
			printf("\n\nTHIS MUST BE DONE ONLY IN LINUX\n\n\n");
			_testcase_creation = true;
			_testcase_multiplier = atof(argv[i+1]);
			i += 2;
			continue;
		}

		printf("Error:\tParameter #%d '%s' not recognized.\n", i, argv[i]);
		exit(1);
	} // while;
	
	return result;
}

bool SFRA::search_n_fps_floorplans( vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps) 
{
	// here we basically run the floorplanner _n_fps times and record the
	// _n_best ones only; the best ones will be used outside this function to
	// simulate them using the vNOC simulator;

	bool result = true;

	//#########################################
	//# Try a 'n_fps' number of floorplanings #
	//#########################################

	B_Tree *fp_p;
	vector<ONE_OF_THE_BEST_FLOOR_PLANS>::iterator pos_Iterator;
	ONE_OF_THE_BEST_FLOOR_PLANS temp_best_floor_plan;
	Module_Info temp_module;
	double fp_aspect_ratio = 1.0;
	int num_rejected_fps = 0;
	int THRESHOLD_TRIALS_COUNT_PER_ATTEMPT = 6;

	// loop _n_fps times and run the BTree floorplanner; record the best ones;
	for ( int i = 1; i <= _n_fps; i++) {
		
		//#######################################
		//# B*Tree representation and Annealing #
		//#######################################

		printf("\n\n\n\n\n\nFloorplan annealing attempt number: %d\n\n", i);

		// (1) load the original floorplanning and apply the annealing;
		fp_p = new B_Tree( _alpha);
		fp_p->set_sfra_host( this); // place a copy of this host in the fp object;
		fp_p->read( _inputfile, _fp_scale);
		_modules_N = fp_p->modules_N;


		// (2) if 2D mode: expand the cores to allocate routers in the same plane;
		// every core - regardless of its initial size - is expanded with a constant
		// amount of area dedicated to implementing the router and network interface;
		if ( _sim_mode == SIMULATED_ARCH_2D) {
			printf("\nModules are now expanded for 2D simulation...\n");
			for ( int id = 0; id < _modules_N; id++) {
				fp_p->inflate_core( id, 1);
			}
		}
		if ( _verbose) {
			fp_p->show_modules(); // debug;
		}
		fp_p->init();


		// (3) run floorplaning;
		SA_FLOORPLANING sa_floorplanning( fp_p, _times, _local, _term_temp);
		sa_floorplanning.set_avg_ratio( _avg_ratio);
		sa_floorplanning.set_lambda( _lambda);
		sa_floorplanning.set_alpha( _alpha);
		sa_floorplanning.set_fp_scale( _fp_scale);
		sa_floorplanning.set_verbose(_verbose); // print detailed info?
		sa_floorplanning.run_SA_Floorplaning(); // run floorplanner;


		// (4)
		fp_p->list_information();
		fp_p->show_tree();

		// (5) compute aspect ratio of this fp;
		fp_aspect_ratio = ( fp_p->Height >= fp_p->Width) ?
			( fp_p->Height / fp_p->Width) : ( fp_p->Width / fp_p->Height);
		// if the aspect ration is higher than 1.3 reject this floorplan;
		if ( num_rejected_fps <= THRESHOLD_TRIALS_COUNT_PER_ATTEMPT &&
			fp_aspect_ratio > 1.3) {
			num_rejected_fps ++;
			if ( num_rejected_fps == _n_fps) {
				// I do this because for hp testcase because cores are
				// thin and tall rects, the floorplanner cannot find
				// floorplans with aspect ratio closed to 1; hence, I 
				// relax the wirelength requirement;
				_alpha = fmin( _alpha + 0.5, 1);
			}
			// cancel this attempt and do not count it as part of _n_fps;
			// however, this should not be done too many (infinite) times;
			i --;
			delete fp_p;
			continue;
		}
		// reset num_rejected_fps so that during next "i" iteration we'll
		// explore more floorplans until we get one with good aspect ratio;
		num_rejected_fps = 0;


		//#################################################
		//# Verify if the resulting FP is one of the best #
		//#################################################

		bool this_fp_is_better = false; // should go into bests list;
		double worst_crit = 0;
		int pos_best, pos_worst, erase_pos;
		double aspect_ratio = 1.0;

		// start filling the list...
		if ( best_fps.size() < _n_best) {
			this_fp_is_better = true; // first three fp's go into list anyway;
			pos_best = -1; // signals any position, just use push_back;
		} else {
			// if list has already a number of n_bests, find the 
			// "worst of the best" to be potentially replaced with the new one;
			for ( int pos = 0; pos < best_fps.size(); pos++) {
				aspect_ratio = 
					( best_fps[pos].Height >= best_fps[pos].Width) ?
					( best_fps[pos].Height / best_fps[pos].Width) :
					( best_fps[pos].Width / best_fps[pos].Height);
				switch ( _fp_criteria) {
				case 'W':
					if (aspect_ratio > 1.3 || best_fps[pos].WireLength > worst_crit) {
						worst_crit = best_fps[pos].WireLength;
						pos_worst = pos;
					}
					break;
				case 'A':
					if (aspect_ratio > 1.3 || best_fps[pos].Area > worst_crit) {
						worst_crit = best_fps[pos].Area;
						pos_worst = pos;
					}
					break;
				default:
					assert(false);
				}
			}
			// compute also the aspect ratio of the worst floorplan in 
			// current bests list;
			aspect_ratio = 
				( best_fps[pos_worst].Height >= best_fps[pos_worst].Width) ?
				( best_fps[pos_worst].Height / best_fps[pos_worst].Width) :
				( best_fps[pos_worst].Width / best_fps[pos_worst].Height);
			// after scaning the whole bests list, if the actual is better than
			// the "worst of the best", assign it to the list;
			switch ( _fp_criteria) {
			case 'W':
				if ( (fp_p->getWireLength() < best_fps[pos_worst].WireLength) ||
					(fp_p->getWireLength() < 1.5*best_fps[pos_worst].WireLength &&
					fp_aspect_ratio < aspect_ratio) ) {
					pos_best = pos_worst;
				 	this_fp_is_better = true;
				}
				break;
			case 'A':
				if ( (fp_p->getArea() < best_fps[pos_worst].Area) ||
					(fp_p->getArea() < 1.5*best_fps[pos_worst].Area &&
					fp_aspect_ratio < aspect_ratio)) {
					pos_best = pos_worst;
					this_fp_is_better = true;
				}
				break;
			}
		}

		if ( this_fp_is_better) {
			printf("\nAttempt number %d goes to the list of best floorplans...\n" ,i);

			// copy all the info from this floorplaning to a temporary container;
			temp_best_floor_plan.attempt_n = i;
			temp_best_floor_plan.Width = fp_p->Width;
			temp_best_floor_plan.Height = fp_p->Height;
			temp_best_floor_plan.Area = fp_p->getArea();
			temp_best_floor_plan.WireLength = fp_p->getWireLength();
			temp_best_floor_plan.modules_N = _modules_N;
			for ( int id = 0; id < _modules_N; id++) {
				// copy all info of each module to a temporary module;
				temp_module.x = fp_p->modules_info[id].x;
				temp_module.y = fp_p->modules_info[id].y;
				temp_module.rx = fp_p->modules_info[id].rx;
				temp_module.ry = fp_p->modules_info[id].ry;
				temp_module.rotate = fp_p->modules_info[id].rotate;
				temp_module.flip = fp_p->modules_info[id].flip;
				// push that temp. module to the vector inside the temp. best fp;
				temp_best_floor_plan.modules_info.push_back( temp_module);
			}

			// add this temp. to the vector of best floorplans;
			if ( pos_best == -1)
				best_fps.push_back( temp_best_floor_plan);
			else {
				printf("\nRemoving attempt number %d from the bests list...\n",
					best_fps[pos_worst].attempt_n);
				pos_Iterator = best_fps.begin(); // erase() requires an iterator;
				erase_pos = 0;
				while ( erase_pos < pos_worst) {
					erase_pos ++;
					pos_Iterator ++;
				}
				best_fps.erase( pos_Iterator);
				best_fps.push_back( temp_best_floor_plan);
			}
		}

		// prepare for the next floorplan
		temp_best_floor_plan.modules_info.clear();
		delete fp_p;
	}

	return result;
}

bool SFRA::routers_assignment_and_vNOC_simulation(
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps)
{
	// here we take every best floorplan and do router assignment - using
	// the Hungarian algo - and then simulate it using the vNOC simulator;
	bool result = true;

	//###########################################################
	//# Read the list of best FP's, assign routers and simulate #
	//###########################################################

	double square_side;	// physical size of the 2-cube mesh;
	long sx, sy; // starting position of the 2-cube mesh;

	for ( int pos = 0; pos < best_fps.size(); pos++) {
		printf("\n\n\n\n\n\nSimulating attempt number: %d.\n\n",
			best_fps[pos].attempt_n);

		// PART A

		//#####################
		//# Router Assignment #
		//#####################
	
		// (1) define a square mesh starting at (0,0)
		sx = 0; sy = 0;
		if ( best_fps[pos].Width < best_fps[pos].Height ) { // take max or min?
			square_side = best_fps[pos].Width;
		} else { 
			square_side = best_fps[pos].Height;
		}
		
		// (2) this is a 2 cube, N ary mesh;
		_ary = (int) ceil( sqrt( _modules_N)) + _x_ary;
		printf("Mesh type: %dx%d\n\n", _ary, _ary);

		ROUTERS_DISTRIBUTION router_distribution(
			_ary, _ary, sx, sy, square_side/(_ary - 1));

		// (3) create an empty router assignment bidimensional table;
		ROUTER_ASSIGNMENT router_assignment( _ary);

		printf("Starting routers assignment...\n\n");
	
		// corners identification;
		//		   NW---NE (rx,ry)
		//		   |	 |
		//		   |	 |				<- for each modules_info[]
		//	 (x,y) SW---SE
		// 
		// 0,0
	
		double dist_a, dist_b, dist_c, dist_d;
		double min_dist_a, min_dist_b, min_dist_c, min_dist_d;
		double min_dist;
		long min_a_x, min_b_x, min_c_x, min_d_x;
		long min_a_y, min_b_y, min_c_y, min_d_y;
	


		// (4) do router assignment using the Hungarian algorithm;

		// (a) create Hungarian object;
		HUNGARIAN_ONE hungarian; // will solve the linear assignment problem;
		int total_number_of_routers = _ary * _ary;
		hungarian.initialize( _modules_N, total_number_of_routers); 

		// (b) for each core:
		int max_x = _ary - 1;
		int max_y = _ary - 1;
		for ( int id = 0; id < _modules_N; id++) {
	
			double dist_a, dist_b, dist_c, dist_d;
			double min_dist;

			// sweep through the routers...
			min_dist_a = -1.0; min_dist_b = -1.0; 
			min_dist_c = -1.0; min_dist_d = -1.0;
			for ( int y = 0; y < _ary; y++) {
				for ( int x = 0; x < _ary; x++) {
					// router at (x,y) location/address now available here;
						
					// Hungarian is looking at the corners of the cores, not
					// their centers
					dist_a = 
						fabs( best_fps[pos].modules_info[id].x - (sx+square_side*x/max_x) ) + 
						fabs( best_fps[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_b = 
						fabs( best_fps[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_c =
						fabs( best_fps[pos].modules_info[id].x - (sx+square_side*x/max_x) ) +
						fabs( best_fps[pos].modules_info[id].ry - (sy+square_side*y/max_y) );
					dist_d =
						fabs( best_fps[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps[pos].modules_info[id].ry - (sy+square_side*y/max_y) );

					min_dist = min(dist_a, min(dist_b, min(dist_c, dist_d)));

					int j = x + y * (max_x + 1);
						
					// min_dist plays the role of cost of assigning
					// this router to this ip;
					hungarian.set_cost( id, j, long(min_dist));
				}
			}
		}
		//hungarian.print_hungarian_assignment(); // debug;

		// (c) call the actual magyar man;
		hungarian.run_hungarian();

		// (d) get assignments;
		for ( int id = 0; id < _modules_N; id++) {
			long assigned_j = hungarian.report_assignment_of( id);
			// hint: assigned_j = x + y * max_x;
			int new_x = assigned_j % (max_x + 1);
			int new_y = assigned_j / (max_x + 1);

			// now that each core (id) already has a respective router
			// (new_x and new_y) find the distances for each corner...
			// corner "SW"
			dist_a =
				fabs( best_fps[pos].modules_info[id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps[pos].modules_info[id].y - (sy+square_side*new_y/max_y) );
			// corner "SE"
			dist_b =
				fabs( best_fps[pos].modules_info[id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps[pos].modules_info[id].y - (sy+square_side*new_y/max_y) );
			// corner "NW"
			dist_c = 
				fabs( best_fps[pos].modules_info[id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps[pos].modules_info[id].ry - (sy+square_side*new_y/max_y) );
			// corner "NE"
			dist_d =
				fabs( best_fps[pos].modules_info[id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps[pos].modules_info[id].ry - (sy+square_side*new_y/max_y) );
	
			min_dist = ceil( min(dist_a, min(dist_b, min(dist_c, dist_d))));

			//...and finally assign this core to that router.
			if (dist_a <= dist_b && dist_a <= dist_c && dist_a <= dist_d ) {
				printf("%d [%c] -> Corner 'SW' to router: %d,%d",
					   id, 48+id, new_x, new_y);
				router_assignment.assign(new_x, new_y, id, SW, min_dist);
			} else { // else is needed because 2 or more equal distances might appear;

				if (dist_b <= dist_a && dist_b <= dist_c && dist_b <= dist_d ) {
					printf("%d [%c] -> Corner 'SE' to router: %d,%d",
						   id, 48+id, new_x, new_y);
					router_assignment.assign(new_x, new_y, id, SE, min_dist);
				} else {

					if (dist_c <= dist_a && dist_c <= dist_b && dist_c <= dist_d ) {
						printf("%d [%c] -> Corner 'NW' to router: %d,%d",
							   id, 48+id, new_x, new_y);
						router_assignment.assign(new_x, new_y, id, NW, min_dist);
					} else {

						if (dist_d <= dist_a && dist_d <= dist_b && dist_d <= dist_c ) {
							printf("%d [%c] -> Corner 'NE' to router: %d,%d",
								   id, 48+id, new_x, new_y);
							router_assignment.assign(new_x, new_y, id, NE, min_dist);
						} else {
							assert(0); 
						}
					}
				}
			}
			printf(" Extra-link: %.01f\n", min_dist);			

		} // for; get assignments;
		if ( _verbose) {
			hungarian.print_hungarian_assignment(); // debug;
		}

	
		// (e) printouts;
		printf("\nRouter assignments summary:\n");
		for ( int y = _ary - 1; y >= 0; y--) {
			for ( int x = 0; x < _ary; x++) {
				printf("\t%d", router_assignment.assigned(x,y));	
			}
			printf("\n");
		}
		printf("Extra-links total: %.01f \n", router_assignment.total_extralinks());

		calculate_average_path_length( &router_assignment); // debug;

 
		// PART B

		//##################
		//# NOC Simulation #
		//##################
	
		// used only by TOPOLOGY objects;
		double link_length = BASE_WIRE; // 1000 um;
		//double link_length  = (BTREE_TO_VNOC_SCALE_MULTIPLIER * square_side) / (_ary - 1);
		printf("Link length: %.01f", link_length);
		printf("\n\n");

		// initially use the original buffers size;
		int buffer_multiplier = 1;
		do {


			RESULTS dummy_list;
			best_fps[pos].results.push_back( dummy_list);
			// get the injection load from the command line;
			int load = _inj_load;
			// if a load sweep is going to be done, start at 10%;
			if ( _load_sweep == true) {
				load = 20;
			}


			do {
				printf("Starting NOC simulation of attempt: %d at load %d%%...\n\n",
					   best_fps[pos].attempt_n, load);

				_skipped_packets = (100 - load) / 10; // with 100% load;
				_max_skip_counter = (10 - _skipped_packets);

				// ctor of VNOC_APPLICATION	does everythingl calls the vNOC
				// simulator, etc.; also puts the final latency result in
				// _sketch_latency of sfra;	
				VNOC_APPLICATION vnoc_app(this, // pass the SFRA object as its host/owner;
										  &router_assignment, _ary, 
										  _inp_buf * buffer_multiplier,
										  _out_buf * buffer_multiplier, 
										  _vc_n, _flit_size, _link_bw, link_length, 
										  _pipeline_in_link, _inputfile, _seed,
										  _routing_a, _cycles, _warmup_cycles, _use_gui,
										  &router_distribution, _gui_pauses,
										  &best_fps[pos]);

				RESULT temp_result;
				temp_result.load = load;
				// retrieve latency as final result of the vNOC simulator;
				temp_result.latency = _sketch_latency;
				temp_result.packets_per_cycle = _sketch_packets_per_cycle;
				temp_result.buffer_multiplier = buffer_multiplier;
				// with a single injection load the latency will be stored at
				// best_fps[pos].results[0].latency
				// if a load sweep is done, we'll have results for 10% at results[0],
				// 20% at results[1], 30% at results[2], ...
				best_fps[pos].results[buffer_multiplier-1].push_back( temp_result);

				// prepare for the next simulation if a load sweep is being done;
				load = load + 10;
			} while ( _load_sweep && load <= 100);

			// prepare for the next round if a buffer size sweep is being done also;
			buffer_multiplier ++;

		} while ( _buffer_sweep && buffer_multiplier <= 5);

	}

	return result;
}

bool SFRA::calculate_final_results_statistics( int argc,char **argv,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps)
{
	// does also printouts and prints in excel file;
	//
	// results format:
	//
	//		   Attempt (0=#1, 1=#2, ...)
	//			  |
	//			  |
	//			  |		 buffer size (0=1x, 1=2x, ...)
	//			  |			   |
	//			  |			   |
	//			  |			   |  Injection load (0=10%, 1=20%, 2=30%, ...)
	//			  |			   |	|
	//			  |			   |	|
	//			  V			   V	V
	//	best_fps[ * ].results[ * ][ * ].latency				[ns]
	//					"			   .load				(10,20,30,...100)
	//					"			   .buffer_multiplier	(1,2,...5)
	//
	bool result = true;



	// (1) print a tag ("name") to the results.txt file for easy identification;
	// we save results in "results/results.txt"; this should be controlled by user?
	FILE *p_results_file;
	char results_filename[268];

	p_results_file = fopen("results/results.txt", "a");
	fprintf(p_results_file, "%s:\n===============================\n", &_test_name);
	// print the parameters to the results.txt file for extra information and
	// future replication;
	for ( int i = 0; i < argc; i++) {
		fprintf(p_results_file, "%s ", argv[i]);
	}
	fprintf(p_results_file, "\n\n");



	// (2) spreadsheet output;
	FILE *p_excel;
	char excelname[281];
	// temp storage; will be flushed to excel file;
	vector<double> sketch_packets_per_cycle;

	if ( _use_excel) {
		sprintf( excelname, "results/%s.xls", &_test_name);
		p_excel = fopen( excelname, "w");
		if ( _load_sweep) {
			fprintf(p_excel, "Buffers\\Load[%]\t20\t30\t40\t50\t60\t70\t80\t90\t100\n");
		} else {
			fprintf(p_excel, "Buffers\\Load[%]\t%d\n", _inj_load);
		}
	}

	printf("RESULTS SUMMARY:\n================\n\n");

	// initially use the original buffers size;
	int buffer_multiplier = 1;

	// loop for several buffer sizes;
	do {

		sketch_packets_per_cycle.clear();
		// first column in the spreadsheet;
		if ( _use_excel) {
			fprintf( p_excel, "%dx\t", buffer_multiplier);
		}
		
		// index is 0 for a single buffer simulation; in sweep 0->1x, 1->2x, 2->3x...
		int buf_sweep_i = buffer_multiplier - 1;
		printf("BUFFERS SIZE: %dx\n\n", buffer_multiplier);
		fprintf(p_results_file, "BUFFERS SIZE: %dx\n\n", buffer_multiplier);

		// index is 0 for a single injection load; in sweep 0->10%, 1->20%, 2->30%...
		int load_sweep_i = 0;
		// get the injection load from the command line;
		int load = _inj_load;
		// if a load sweep is going to be done, start at 10%
		if ( _load_sweep == true) {
			load = 20;
		}


		// loop for several injection loads;
		do {
			double latency_sum = 0, avg_latency, min_latency = DBL_MAX;
			double std_deviation, squared_deviations_sum = 0;
			double packets_per_cycle_of_min_latency = 0;

			packets_per_cycle_of_min_latency =
				best_fps[0].results[0][load_sweep_i].packets_per_cycle;
			printf("INJECTION LOAD: %d%%  packets/cycle: %.4lf \n",
				   load, packets_per_cycle_of_min_latency);
			fprintf(p_results_file, "INJECTION LOAD: %d%%  packets/cycle: %.4lf \n",
					load, packets_per_cycle_of_min_latency);
			// record packets_per_cycle; will be flushed to excell file too;
			sketch_packets_per_cycle.push_back( packets_per_cycle_of_min_latency);

			for (int pos = 0; pos < best_fps.size(); pos++) {

				// verify if the indexing for a load sweep and/or 
				// buffer sweep is right;
				assert( load == 
					best_fps[pos].results[buf_sweep_i][load_sweep_i].load);
				assert( buffer_multiplier == 
					best_fps[pos].results[buf_sweep_i][load_sweep_i].buffer_multiplier);

				// calculate the sum of all latencies (for this load %) 
				// and minimum;
				latency_sum += best_fps[pos].results[buf_sweep_i][load_sweep_i].latency;
				printf("Attempt: %d\tFlit latency: %.02lf\n",
					best_fps[pos].attempt_n, 
					best_fps[pos].results[buf_sweep_i][load_sweep_i].latency);
				fprintf(p_results_file, "Attempt: %d\tFlit latency: %.02lf\n",
					best_fps[pos].attempt_n,
					best_fps[pos].results[buf_sweep_i][load_sweep_i].latency);
				min_latency = min(min_latency, 
					best_fps[pos].results[buf_sweep_i][load_sweep_i].latency);
			}

			// calculate the average latency and square deviation;
			avg_latency = latency_sum / best_fps.size();
			for (int pos=0; pos<best_fps.size() ;pos++) {
				squared_deviations_sum += pow(best_fps[pos].results[buf_sweep_i][load_sweep_i].latency - avg_latency,2);
			}
			std_deviation = sqrt( squared_deviations_sum / best_fps.size());

			printf("\nAverage flit latency: %.02lf\n", avg_latency);
			fprintf(p_results_file,"\nAverage flit latency: %.02lf\n", avg_latency);
			printf("Standard deviation: %.02lf\n", std_deviation);
			fprintf(p_results_file,"Standard deviation: %.02lf\n", std_deviation);
			printf("\nMinimum flit latency: %.02lf\n\n\n\n", min_latency);
			fprintf(p_results_file,"\nMinimum flit latency: %.02lf\n\n\n\n", min_latency);
			if ( _use_excel) {
				fprintf(p_excel,"%.02lf\t", min_latency);
			}

			// prepare for the next summary if a load sweep is being done;
			load = load + 10;
			load_sweep_i ++;

		} while ( _load_sweep && load <= 100);

		// prepare for the next round if a buffer size sweep is being done;
		buffer_multiplier ++;
		// write in the excel file also the packets/cycle values corresponding to 
		// min latencies already written in;
		if ( _use_excel) {
			fprintf(p_excel, "\n");
			for ( int i = 0; i < sketch_packets_per_cycle.size(); i++) {
				fprintf(p_excel,"%.4lf\t", sketch_packets_per_cycle[i]);
			}
		}

	} while ( _buffer_sweep && buffer_multiplier <= 5);


	fclose( p_results_file);
	if ( _use_excel) {
		fclose(p_excel);
	}

	return result;
}

////////////////////////////////////////////////////////////////////////////////
//
// APPLICATION_GRAPH
//
////////////////////////////////////////////////////////////////////////////////
//
// this is the graph that models the application task graph; nodes are tasks
// and edges are communications between tasks labeled with their communication
// volume;
//
////////////////////////////////////////////////////////////////////////////////

void APPLICATION_GRAPH::print_application_graph()
{
	// debug;
	printf("\nApplication graph:");
	printf("\nmax_nodes_comm_volume: %.2lf", _max_nodes_comm_volume);
	printf("\nmax_arcs_comm_volume: %.2lf", _max_arcs_comm_volume);
	printf("\nIP/cores: %d", _nodes.size());
	for ( long i = 0; i < _nodes_count; i ++) {
		printf("\n%d  area: %d  cumul comm vol: %d", i,
			_nodes[i].area(), int(_nodes[i].io_comm_volume()));
		printf("        \tfin:");
		for ( long k = 0; k < _nodes[i].fanin().size(); k ++) {
			printf(" %d", _nodes[i].fanin()[k]);
		}
		printf("\tfout:");
		for ( long k = 0; k < _nodes[i].fanout().size(); k ++) {
			printf(" %d", _nodes[i].fanout()[k]);
		}
		//printf("\tcomm_vol: %.1f", _nodes[i].io_comm_volume());
	}
	printf("\nArcs: %d", _arcs.size());
	for ( long j = 0; j < _arcs_count; j ++) {
		printf("\n%d -- %d    \t%.1f",
			_arcs[j].src_id(), _arcs[j].des_id(), _arcs[j].comm_volume());
	}
	printf("\n");
}

bool APPLICATION_GRAPH::create_nodes_and_arcs( B_Tree *fp_p)
{
	// create application communication graph;

	// (1) nodes;
	int modules_N = fp_p->size();
	for ( int core_id = 0; core_id < modules_N; core_id++) {
		int this_area = fp_p->get_module_area( core_id);
		_nodes.push_back( APPLICATION_NODE( _nodes_count, this_area));
		_nodes_count ++; // prepare for next one;
	}
	

	// (2) arcs;
	// Note; should be different val for diff testcase, based on how testcases
	// were created?
	double comm_multiplier = 1.0;
	for( int i = 0; i < modules_N; i++) {
		for( int j = 0; j < modules_N; j++) {
			int this_connection = fp_p->get_connection( i, j);

			if ( i != j && this_connection > 0) { // 2 is to ignore power/gnd;
				double this_comm_volume = this_connection * comm_multiplier;

				_nodes[ i].add_fanout( j);
				_nodes[ j].add_fanin( i);

				_arcs.push_back( APPLICATION_ARC( _arcs_count, i, j, this_comm_volume));
				_arcs_count ++; // prepare for next one;
			}
		}
	}
	
	// (3) misc stuff;
	compute_max_nodes_comm_volume();
	compute_max_arcs_comm_volume();

	return true;
}

void APPLICATION_GRAPH::init_hmetis_interface()
{
	// used for transfering nodes and nets to hmetis interface in
	// preparation for calling hmetis partitioner; adds in all
	// nodes and edges from the outside world (host);

	//printf("\nInitialize hmetis interface.");
	int local_num_nets = _arcs_count;
	int local_num_blocks = _nodes_count;
	int this_node_partition = -1;

	// (1) instatiate the Metis Interface class;
	_hmetis_interface = new MetisIntfc( local_num_blocks, local_num_nets);

	// (2.a) add Modules (i.e., nodes, vertices);
	for ( int i = 0; i < _nodes_count; i++) {
		_hmetis_interface->AddModule( i); // id is used to transfer to hmetis all blocks
		_hmetis_interface->setModWeight( i, _nodes[i].area());

		// if this has to be a fixed node in a required partition, then
		// impose this constraint; partition 0..k for k-way partitioning;
		// or 0 or 1 for bi-partitioning;
		this_node_partition = _nodes[i].partition(); // 0, 1, or -1 (free);
		_hmetis_interface->setModPartition( i, this_node_partition);
	}

	// (2.b) fix 10% of largest cores into partitions to have
	// big cores in both partitions; sort IP/cores after their 
	// area and assign them into partitions alternatively;
	bool fix_some_cores_as_anchors = true;
	if ( fix_some_cores_as_anchors == true) {
		vector<PAIR_TWO> nodes_areas;
		for ( int i = 0; i < _nodes_count; i ++) {
			nodes_areas.push_back( PAIR_TWO(i, get_node_area( i)));
		}
		sort( nodes_areas.begin(), nodes_areas.end());
		for ( int i = 0; i < 4; i ++) { // assume there are at least 6 cores;
			// Note: take the last (ie largest) corest from sorted list;
			int id = nodes_areas[ _nodes_count - 1 - i].id();
			if ( (id % 2) > 0) {
				_hmetis_interface->setModPartition( id, 1);
			} else {
				_hmetis_interface->setModPartition( id, 0);
			}
		}
	}

	// (3) add connectivity information: one arc at a time;
	for ( int j = 0; j < _arcs_count; j++) {
		int *mods = new int[ 2 + 1 ]; // each arc has only two terminals;
		mods[ 0] = _arcs[j].src_id();
		mods[ 1] = _arcs[j].des_id();

		_hmetis_interface->AddNet( j, 2, mods);
		_hmetis_interface->setNetWeight( j, _arcs[j].comm_volume());
      
		delete mods;
	}

	//printf("\nDone init_hmetis_interface.");
}

void APPLICATION_GRAPH::get_hmetis_results()
{
	int this_partition = -1;
	for ( int i = 0; i < _nodes_count; i++) {
		this_partition = _hmetis_interface->getModPartition( i );
		_nodes[i].set_partition( this_partition);
	}
}

bool APPLICATION_GRAPH::run_partitioning( int num_partitions)
{
	// initialize hmetis interface, populate it, call hmetis, and put result 
	// back into the caller host;


	// (1) init: put local-host graph into hmetis interface;
	// this is one time deal; later we can perform partitioning as many 
	// times we want; we'll only change the fraction of fixed nodes in
	// each partition;
	init_hmetis_interface();
	if ( _sfra->verbose()) {
		_hmetis_interface->ShowHyperGraph();
	}


	// (2) actual call of hMetis partitioner;
	_hmetis_interface->SetOption( "UBfactor", 8);
	_hmetis_interface->SetOption( "Seed", 1);
	if ( _sfra->verbose()) {
		_hmetis_interface->ShowOptions();
		_hmetis_interface->ShowHyperGraph();
		printf("\nPerforming %d-way partitioning...", num_partitions);
	}
	_hmetis_interface->Partition( num_partitions);
	if ( _sfra->verbose()) {
		printf("\nDone hmetis partitioning.");
		//_hmetis_interface->ShowResults();
	}
		

	// (3) get result of partitioning from interface and put back 
	// into local-host; also record cut_freq of each cut edge;
	get_hmetis_results();
	if ( _sfra->verbose()) { 
		print_graph_partitions(); // debug;
	}
}

void APPLICATION_GRAPH::print_graph_partitions()
{

	_hmetis_interface->ShowResults();
	int this_partition = 0;
	printf("\nPartition 1:  ");
	for ( int i = 0; i < _nodes_count; i++) {
		if ( _nodes[i].partition() == 0)
			printf(" %d", i);
	}
	printf("\nPartition 2:  ");
	for ( int i = 0; i < _nodes_count; i++) {
		if ( _nodes[i].partition() == 1)
			printf(" %d", i);
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// 3D related functions; most of them are developed from those used for
// 2D and 25D above; howver, in 3D architecture we first partition the 
// task graph into 2 and do floorplanning on each separately;
//
////////////////////////////////////////////////////////////////////////////////


bool SFRA::search_n_fps_floorplans_3D(
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2) 
{

	bool result = true;


	//###########################################
	//# Create top level task/application graph #
	//###########################################

	B_Tree *fp_p;

	// (1) load the original floorplan testcase;
	fp_p = new B_Tree( _alpha);
	fp_p->set_sfra_host( this); // place a copy of this host in the fp object;
	fp_p->read( _inputfile, _fp_scale);

	if ( _verbose) {
		fp_p->show_modules(); // debug;
	}
	fp_p->init();

	// Note: in a one time action only create the _application_graph;
	if ( _application_graph.nodes_count() <= 0) {
		printf("\nCreate the application_graph...");
		_application_graph.create_nodes_and_arcs( fp_p);
		_application_graph.set_host( this);
		if ( _verbose) {
			_application_graph.print_application_graph(); // debug;
		}
		printf("\nDone create application_graph.\n");
	}


	//###########################################
	//# Partition and create 2 sub-floorplans   #
	//###########################################
	
	vector<int> partition;
	int appl_nodes_count = _application_graph.nodes_count();
	bool use_hmetis = false;
	if ( use_hmetis == true) {
		// run hMetis partitioner;
		_application_graph.run_partitioning( 2); // num_partitions = 2;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			partition.push_back( _application_graph.get_node_partition( i));
		}
	}
	else {
		for ( int i = 0; i < appl_nodes_count; i ++) {
			partition.push_back( -1);
		}
		// sort IP/cores after their area and assign them into partitions
		// alternatively;
		vector<PAIR_TWO> nodes_areas;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			nodes_areas.push_back( 
				PAIR_TWO(i, _application_graph.get_node_area( i)));
		}
		sort( nodes_areas.begin(), nodes_areas.end()); // small->big;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			int id = nodes_areas[i].id();
			if ( (id % 2) > 0) {
				partition[ id] = 1;
				//printf("%d(%d): 1 ",id,_application_graph.get_node_area( id));
			} else {
				partition[ id] = 0;
				//printf("%d(%d): 1 ",id,_application_graph.get_node_area( id));
			}
		}
	}
	

	// () construct the magic mappings between top-level floorplan
	// modules ids and the ids inside the subfloorplans; need these mappings
	// to be able to jump from ones to the others for assignment and then for 
	// vNOC simulation purposes;
	// let's say that in the top level we have 10 and subfloorplan 1 has 5 
	// cores out of the 10; 
	// _magic_topid_to_subid = {0 -1 -1 1 2 -1 -1 -1 3 4}; top-level node
	// id 9 corresponds to subfloorplan node id 4;
	// _magic_subid_to_topid = {0 3 4 8 9};
	int counter1_i = 0, counter2_i = 0;
	for ( int i = 0; i < appl_nodes_count; i ++) {
		if ( partition[i] == 0) {
			_magic_topid_to_subid.push_back( counter1_i);
			_magic_subid1_to_topid.push_back( i);
			counter1_i ++;
		} else { // partition[i] == 1;
			_magic_topid_to_subid.push_back( counter2_i);
			_magic_subid2_to_topid.push_back( i);
			counter2_i ++;
		}				
	}	


	// () create the two subfloorplan objects;
	B_Tree *fp_p1;
	B_Tree *fp_p2;
	fp_p1 = new B_Tree( _alpha);
	fp_p2 = new B_Tree( _alpha);
	fp_p1->set_sfra_host( this);
	fp_p2->set_sfra_host( this);

	fp_p1->create_subfloorplan( 0, fp_p, partition);
	fp_p2->create_subfloorplan( 1, fp_p, partition);	


	//#########################################
	//# Try a 'n_fps' number of floorplanings #
	//#########################################

	// () best found floorplans will be recorded in the arguments 
	// best_fps1, best_fps2;

	search_n_fps_for_subfloorplan( fp_p1, best_fps1);
 
	search_n_fps_for_subfloorplan( fp_p2, best_fps2);



	// () clean-up;
	delete fp_p1;
	delete fp_p2;
	delete fp_p;

	return result;
}

void SFRA::search_n_fps_for_subfloorplan( B_Tree *fp_p,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps)
{
	// () run the floorplanner n_fps times and record n_best ones;
	// actually it may be more than n_fps in order to seek aspect_ratio
	// closer to 1;
	vector<ONE_OF_THE_BEST_FLOOR_PLANS>::iterator pos_Iterator;
	ONE_OF_THE_BEST_FLOOR_PLANS temp_best_floor_plan;
	Module_Info temp_module;
	double fp_aspect_ratio = 1.0;
	int num_rejected_fps = 0;
	int THRESHOLD_TRIALS_COUNT_PER_ATTEMPT = 6;

	// loop _n_fps times and run the BTree floorplanner; record the best ones;
	for ( int i = 1; i <= _n_fps; i++) {

		//#############
		//# Annealing #
		//#############

		printf("\n\n\n\n\n\nFloorplan annealing attempt number: %d\n\n", i);


		// (1)
		_modules_N = fp_p->modules_N; // modules count of this subfloorplan;
		if ( _verbose) {
			fp_p->show_modules(); // debug;
		}
		fp_p->init();


		// (2) run floorplaning;
		SA_FLOORPLANING sa_floorplanning( fp_p, _times, _local, _term_temp);
		sa_floorplanning.set_avg_ratio( _avg_ratio);
		sa_floorplanning.set_lambda( _lambda);
		sa_floorplanning.set_alpha( _alpha);
		sa_floorplanning.set_fp_scale( _fp_scale);
		sa_floorplanning.set_verbose(_verbose); // print detailed info?
		sa_floorplanning.run_SA_Floorplaning(); // run floorplanner;


		// (3)
		fp_p->list_information();
		fp_p->show_tree();


		// (4) compute aspect ratio of this fp;
		fp_aspect_ratio = ( fp_p->Height >= fp_p->Width) ?
			( fp_p->Height / fp_p->Width) : ( fp_p->Width / fp_p->Height);

		// if the aspect ratio is higher than 1.3 reject this floorplan;
		if ( num_rejected_fps <= THRESHOLD_TRIALS_COUNT_PER_ATTEMPT && 
			fp_aspect_ratio > 1.3) {
			num_rejected_fps ++;
			if ( num_rejected_fps == _n_fps) {
				// I do this because for hp testcase because cores are
				// thin and tall rects, the floorplanner cannot find
				// floorplans with aspect ratio closed to 1; hence, I 
				// relax the wirelength requirement;
				_alpha = fmin( _alpha + 0.5, 1);
			}
			// cancel this attempt and do not count it as part of _n_fps;
			// however, this should not be done too many (infinite) times;
			i --;
			continue;
		}
		// reset num_rejected_fps so that during next "i" iteration we'll
		// explore more floorplans until we get one with good aspect ratio;
		num_rejected_fps = 0;
		

		//#################################################
		//# Verify if the resulting FP is one of the best #
		//#################################################

		bool this_fp_is_better = false; // should go into bests list;
		double worst_crit = 0;
		int pos_best, pos_worst, erase_pos;
		double aspect_ratio = 1.0;

		
		// start filling the list...
		if ( best_fps.size() < _n_best) {
			this_fp_is_better = true; // first three fp's go into list anyway;
			pos_best = -1; // signals any position, just use push_back;
		} else {
			// if list has already a number of n_bests, find the 
			// "worst of the best" to be potentially replaced with the new one;
			for ( int pos = 0; pos < best_fps.size(); pos++) {
				aspect_ratio = 
					( best_fps[pos].Height >= best_fps[pos].Width) ?
					( best_fps[pos].Height / best_fps[pos].Width) :
					( best_fps[pos].Width / best_fps[pos].Height);
				switch ( _fp_criteria) {
				case 'W':
					if (aspect_ratio > 1.3 || best_fps[pos].WireLength > worst_crit) {
						worst_crit = best_fps[pos].WireLength;
						pos_worst = pos;
					}
					break;
				case 'A':
					if (aspect_ratio > 1.3 || best_fps[pos].Area > worst_crit) {
						worst_crit = best_fps[pos].Area;
						pos_worst = pos;
					}
					break;
				default:
					assert(false);
				}
			}
			// compute also the aspect ratio of the worst floorplan in 
			// current bests list;
			aspect_ratio = 
				( best_fps[pos_worst].Height >= best_fps[pos_worst].Width) ?
				( best_fps[pos_worst].Height / best_fps[pos_worst].Width) :
				( best_fps[pos_worst].Width / best_fps[pos_worst].Height);
			// after scaning the whole bests list, if the actual is better than
			// the "worst of the best", assign it to the list;
			switch ( _fp_criteria) {
			case 'W':
				if ( (fp_p->getWireLength() < best_fps[pos_worst].WireLength) ||
					(fp_p->getWireLength() < 1.5*best_fps[pos_worst].WireLength &&
					fp_aspect_ratio < aspect_ratio) ) {
					pos_best = pos_worst;
				 	this_fp_is_better = true;
				}
				break;
			case 'A':
				if ( (fp_p->getArea() < best_fps[pos_worst].Area) ||
					(fp_p->getArea() < 1.5*best_fps[pos_worst].Area &&
					fp_aspect_ratio < aspect_ratio)) {
					pos_best = pos_worst;
					this_fp_is_better = true;
				}
				break;
			}
		}

		if ( this_fp_is_better) {
			printf("\nAttempt number %d goes to the list of best floorplans...\n" ,i);

			// copy all the info from this floorplaning to a temporary container;
			temp_best_floor_plan.attempt_n = i;
			temp_best_floor_plan.Width = fp_p->Width;
			temp_best_floor_plan.Height = fp_p->Height;
			temp_best_floor_plan.Area = fp_p->getArea();
			temp_best_floor_plan.WireLength = fp_p->getWireLength();
			temp_best_floor_plan.modules_N = _modules_N;
			for ( int id = 0; id < _modules_N; id++) {
				// copy all info of each module to a temporary module;
				temp_module.x = fp_p->modules_info[id].x;
				temp_module.y = fp_p->modules_info[id].y;
				temp_module.rx = fp_p->modules_info[id].rx;
				temp_module.ry = fp_p->modules_info[id].ry;
				temp_module.rotate = fp_p->modules_info[id].rotate;
				temp_module.flip = fp_p->modules_info[id].flip;
				// push that temp. module to the vector inside the temp. best fp;
				temp_best_floor_plan.modules_info.push_back( temp_module);
			}

			// add this temp. to the vector of best floorplans;
			if ( pos_best == -1)
				best_fps.push_back( temp_best_floor_plan);
			else {
				printf("\nRemoving attempt number %d from the bests list...\n",
					best_fps[pos_worst].attempt_n);
				pos_Iterator = best_fps.begin(); // erase() requires an iterator;
				erase_pos = 0;
				while ( erase_pos < pos_worst) {
					erase_pos ++;
					pos_Iterator ++;
				}
				best_fps.erase( pos_Iterator);
				best_fps.push_back( temp_best_floor_plan);
			}
		}

		// prepare for the next floorplan
		temp_best_floor_plan.modules_info.clear();
	}
}

bool SFRA::routers_assignment_and_vNOC_simulation_3D(
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2)
{
	// here we take every two best floorplans, overlap them and do 
	// router assignment - using the Hungarian algo - and then simulate 
	// it using the vNOC simulator;
	// the routers mesh NOC is created with dimensions given by maximum
	// subfloorplan in x and y directions;
	bool result = true;

	//###########################################################
	//# Read the list of best FP's, assign routers and simulate #
	//###########################################################

	double square_side;	// physical size of the 2-cube mesh;
	long sx, sy; // starting position of the 2-cube mesh;

	int best_fps_count = best_fps1.size();
	
	for ( int pos = 0; pos < best_fps_count; pos++) {
		printf("\n\n\n\n\n\nSimulating attempt numbers: %d, %d.\n\n",
			best_fps1[pos].attempt_n, best_fps2[pos].attempt_n);

		_modules_N = best_fps1[pos].modules_N + best_fps2[pos].modules_N;


		// PART A

		//#####################
		//# Router Assignment #
		//#####################
	

		// (1) define a square mesh starting at (0,0)
		sx = 0; sy = 0;
		double max_width = fmax( best_fps1[pos].Width, best_fps2[pos].Width);
		double max_height = fmax( best_fps1[pos].Height, best_fps2[pos].Height);
		square_side = fmin( max_width, max_height); // take max or min?
		

		// (2) this is a 2 cube, N ary mesh;
		_ary = (int) ceil( sqrt( _modules_N)) + _x_ary;
		printf("Mesh type: %dx%d\n\n", _ary, _ary);

		ROUTERS_DISTRIBUTION router_distribution(
			_ary, _ary, sx, sy, square_side/(_ary - 1));


		// (3) create an empty router assignment bidimensional table;
		ROUTER_ASSIGNMENT router_assignment( _ary);

		printf("Starting routers assignment...\n\n");
	
		// corners identification;
		//		   NW---NE (rx,ry)
		//		   |	 |
		//		   |	 |				<- for each modules_info[]
		//	 (x,y) SW---SE
		// 
		// 0,0
	
		double dist_a, dist_b, dist_c, dist_d;
		double min_dist_a, min_dist_b, min_dist_c, min_dist_d;
		double min_dist;
		long min_a_x, min_b_x, min_c_x, min_d_x;
		long min_a_y, min_b_y, min_c_y, min_d_y;
	


		// (4) do router assignment using the Hungarian algorithm;


		// (a) create Hungarian object;
		HUNGARIAN_ONE hungarian; // will solve the linear assignment problem;
		int total_number_of_routers = _ary * _ary;
		hungarian.initialize( _modules_N, total_number_of_routers); 


		// (b) take each core of the two subfloorplans and construct
		// the hungarian object;
		int max_x = _ary - 1;
		int max_y = _ary - 1;


		// () process first the cores of subfloorplan 1;
		int modules_N_1 = best_fps1[pos].modules_N;
		for ( int id = 0; id < modules_N_1; id++) {
			double dist_a, dist_b, dist_c, dist_d;
			double min_dist;

			// sweep through the routers...
			min_dist_a = -1.0; min_dist_b = -1.0; 
			min_dist_c = -1.0; min_dist_d = -1.0;
			for ( int y = 0; y < _ary; y++) {
				for ( int x = 0; x < _ary; x++) {
					// router at (x,y) location/address now available here;
					// Hungarian is looking at the corners of the cores, not
					// their centers
					dist_a = 
						fabs( best_fps1[pos].modules_info[id].x - (sx+square_side*x/max_x) ) + 
						fabs( best_fps1[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_b = 
						fabs( best_fps1[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps1[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_c =
						fabs( best_fps1[pos].modules_info[id].x - (sx+square_side*x/max_x) ) +
						fabs( best_fps1[pos].modules_info[id].ry - (sy+square_side*y/max_y) );
					dist_d =
						fabs( best_fps1[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps1[pos].modules_info[id].ry - (sy+square_side*y/max_y) );

					min_dist = min(dist_a, min(dist_b, min(dist_c, dist_d)));

					int j = x + y * (max_x + 1);
						
					// min_dist plays the role of cost of assigning
					// this router to this ip;
					hungarian.set_cost( id, j, long(min_dist));
				}
			}
		}
		// () process first the cores of subfloorplan 2;
		int modules_N_2 = best_fps2[pos].modules_N;
		for ( int id = 0; id < modules_N_2; id++) {
			double dist_a, dist_b, dist_c, dist_d;
			double min_dist;

			// sweep through the routers...
			min_dist_a = -1.0; min_dist_b = -1.0; 
			min_dist_c = -1.0; min_dist_d = -1.0;
			for ( int y = 0; y < _ary; y++) {
				for ( int x = 0; x < _ary; x++) {
					// router at (x,y) location/address now available here;
					// Hungarian is looking at the corners of the cores, not
					// their centers
					dist_a = 
						fabs( best_fps2[pos].modules_info[id].x - (sx+square_side*x/max_x) ) + 
						fabs( best_fps2[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_b = 
						fabs( best_fps2[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps2[pos].modules_info[id].y - (sy+square_side*y/max_y) );
					dist_c =
						fabs( best_fps2[pos].modules_info[id].x - (sx+square_side*x/max_x) ) +
						fabs( best_fps2[pos].modules_info[id].ry - (sy+square_side*y/max_y) );
					dist_d =
						fabs( best_fps2[pos].modules_info[id].rx - (sx+square_side*x/max_x) ) +
						fabs( best_fps2[pos].modules_info[id].ry - (sy+square_side*y/max_y) );

					min_dist = min(dist_a, min(dist_b, min(dist_c, dist_d)));

					int j = x + y * (max_x + 1);
						
					// min_dist plays the role of cost of assigning
					// this router to this ip;
					// Note: the modules of this second subfloorplan will be indexed as
					// their index + modules_N_1 in order to distinguish them from modules
					// of the first subfloorplan; this biased indexing acts anyways inside
					// the hungarian object;
					hungarian.set_cost( (id + modules_N_1), j, long(min_dist));
				}
			}
		}



		// (c) call the actual magyar man;
		//hungarian.print_hungarian_assignment(); // debug;
		hungarian.run_hungarian();
		//hungarian.print_hungarian_assignment(); // debug;



		// (d) get assignments;

		// () process first the cores of subfloorplan 1;
		for ( int id = 0; id < modules_N_1; id++) {
			long assigned_j = hungarian.report_assignment_of( id);
			// hint: assigned_j = x + y * max_x;
			int new_x = assigned_j % (max_x + 1);
			int new_y = assigned_j / (max_x + 1);

			// now that each core (id) already has a respective router
			// (new_x and new_y) find the distances for each corner...
			// corner "SW"
			dist_a =
				fabs( best_fps1[pos].modules_info[id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps1[pos].modules_info[id].y - (sy+square_side*new_y/max_y) );
			// corner "SE"
			dist_b =
				fabs( best_fps1[pos].modules_info[id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps1[pos].modules_info[id].y - (sy+square_side*new_y/max_y) );
			// corner "NW"
			dist_c = 
				fabs( best_fps1[pos].modules_info[id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps1[pos].modules_info[id].ry - (sy+square_side*new_y/max_y) );
			// corner "NE"
			dist_d =
				fabs( best_fps1[pos].modules_info[id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps1[pos].modules_info[id].ry - (sy+square_side*new_y/max_y) );
	
			min_dist = ceil( min(dist_a, min(dist_b, min(dist_c, dist_d))));

			//...and finally assign this core to that router.
			// Note: use the top-level id; will later be used inside vNOC to
			// identify the tol-level core id, accordig to which the trace files,
			// used to inject packets, were generated;
			int top_id = _magic_subid1_to_topid[ id];
			
			if (dist_a <= dist_b && dist_a <= dist_c && dist_a <= dist_d ) {
				printf("%d [%c] -> Corner 'SW' to router: %d,%d",
					   top_id, 48+top_id, new_x, new_y);
				router_assignment.assign(new_x, new_y, top_id, SW, min_dist);
			} else { // else is needed because 2 or more equal distances might appear;

				if (dist_b <= dist_a && dist_b <= dist_c && dist_b <= dist_d ) {
					printf("%d [%c] -> Corner 'SE' to router: %d,%d",
						   top_id, 48+top_id, new_x, new_y);
					router_assignment.assign(new_x, new_y, top_id, SE, min_dist);
				} else {

					if (dist_c <= dist_a && dist_c <= dist_b && dist_c <= dist_d ) {
						printf("%d [%c] -> Corner 'NW' to router: %d,%d",
							   top_id, 48+top_id, new_x, new_y);
						router_assignment.assign(new_x, new_y, top_id, NW, min_dist);
					} else {

						if (dist_d <= dist_a && dist_d <= dist_b && dist_d <= dist_c ) {
							printf("%d [%c] -> Corner 'NE' to router: %d,%d",
								   top_id, 48+top_id, new_x, new_y);
							router_assignment.assign(new_x, new_y, top_id, NE, min_dist);
						} else {
							assert(0); 
						}
					}
				}
			}
			printf(" Extra-link: %.01f\n", min_dist);			

		} // for; get assignments;
		// () process first the cores of subfloorplan 2;
		for ( int id = modules_N_1; id < _modules_N; id++) {
			long assigned_j = hungarian.report_assignment_of( id);
			// hint: assigned_j = x + y * max_x;
			int new_x = assigned_j % (max_x + 1);
			int new_y = assigned_j / (max_x + 1);

			// now that each core (id) already has a respective router
			// (new_x and new_y) find the distances for each corner...
			// corner "SW"
			// Note: here we have to use the subfloorplan module id, derived from
			// the index used inside the hungarian object;
			int sub_id = id - modules_N_1;
			dist_a =
				fabs( best_fps2[pos].modules_info[sub_id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps2[pos].modules_info[sub_id].y - (sy+square_side*new_y/max_y) );
			// corner "SE"
			dist_b =
				fabs( best_fps2[pos].modules_info[sub_id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps2[pos].modules_info[sub_id].y - (sy+square_side*new_y/max_y) );
			// corner "NW"
			dist_c = 
				fabs( best_fps2[pos].modules_info[sub_id].x - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps2[pos].modules_info[sub_id].ry - (sy+square_side*new_y/max_y) );
			// corner "NE"
			dist_d =
				fabs( best_fps2[pos].modules_info[sub_id].rx - (sx+square_side*new_x/max_x) ) +
				fabs( best_fps2[pos].modules_info[sub_id].ry - (sy+square_side*new_y/max_y) );
	
			min_dist = ceil( min(dist_a, min(dist_b, min(dist_c, dist_d))));

			//...and finally assign this core to that router.
			// Note: use the top-level id; will later be used inside vNOC to
			// identify the tol-level core id, accordig to which the trace files,
			// used to inject packets, were generated;
			int top_id = _magic_subid2_to_topid[ sub_id];

			if (dist_a <= dist_b && dist_a <= dist_c && dist_a <= dist_d ) {
				printf("%d [%c] -> Corner 'SW' to router: %d,%d",
					   top_id, 48+top_id, new_x, new_y);
				router_assignment.assign(new_x, new_y, top_id, SW, min_dist);
			} else { // else is needed because 2 or more equal distances might appear;

				if (dist_b <= dist_a && dist_b <= dist_c && dist_b <= dist_d ) {
					printf("%d [%c] -> Corner 'SE' to router: %d,%d",
						   top_id, 48+top_id, new_x, new_y);
					router_assignment.assign(new_x, new_y, top_id, SE, min_dist);
				} else {

					if (dist_c <= dist_a && dist_c <= dist_b && dist_c <= dist_d ) {
						printf("%d [%c] -> Corner 'NW' to router: %d,%d",
							   top_id, 48+top_id, new_x, new_y);
						router_assignment.assign(new_x, new_y, top_id, NW, min_dist);
					} else {

						if (dist_d <= dist_a && dist_d <= dist_b && dist_d <= dist_c ) {
							printf("%d [%c] -> Corner 'NE' to router: %d,%d",
								   top_id, 48+top_id, new_x, new_y);
							router_assignment.assign(new_x, new_y, top_id, NE, min_dist);
						} else {
							assert(0); 
						}
					}
				}
			}
			printf(" Extra-link: %.01f\n", min_dist);			

		} // for; get assignments;


		if ( _verbose) {
			//hungarian.print_hungarian_assignment(); // debug;
		}
		
	
		// (e) printouts;
		printf("\nRouter assignments summary:\n");
		for ( int y = _ary - 1; y >= 0; y--) {
			for ( int x = 0; x < _ary; x++) {
				printf("\t%d", router_assignment.assigned(x,y));	
			}
			printf("\n");
		}
		printf("Extra-links total: %.01f \n", router_assignment.total_extralinks());

		
		// PART B

		//##################
		//# NOC Simulation #
		//##################
	
		// used only by TOPOLOGY objects;
		double link_length = BASE_WIRE; // 1000 um;
		//double link_length = (BTREE_TO_VNOC_SCALE_MULTIPLIER * square_side) / (_ary - 1);
		printf("Link length: %.01f", link_length);
		printf("\n\n");
	
		// initially use the original buffers size;
		int buffer_multiplier = 1;
		do {


			RESULTS dummy_list;
			// Note: I will record vNOC simulation results in the nest_fps1 objects;
			// these results will represent the results of the 3D architecture 
			// simulation for the top-level application with cores placed on 
			// layers 1 and 3;
			best_fps1[pos].results.push_back( dummy_list);
			// get the injection load from the command line;
			int load = _inj_load;
			// if a load sweep is going to be done, start at 10%;
			if ( _load_sweep == true) {
				load = 20;
			}


			do {
				printf("Starting NOC simulation of attempt: %d at load %d%%...\n\n",
					   best_fps1[pos].attempt_n, load);

				_skipped_packets = (100 - load) / 10; // with 100% load;
				_max_skip_counter = (10 - _skipped_packets);

				// ctor of VNOC_APPLICATION	does everythingl calls the vNOC
				// simulator, etc.; also puts the final latency result in
				// _sketch_latency of sfra;	
				VNOC_APPLICATION vnoc_app(this, // pass the SFRA object as its host/owner;
										  &router_assignment, _ary, 
										  _inp_buf * buffer_multiplier,
										  _out_buf * buffer_multiplier, 
										  _vc_n, _flit_size, _link_bw, link_length, 
										  _pipeline_in_link, _inputfile, _seed,
										  _routing_a, _cycles, _warmup_cycles, _use_gui,
										  &router_distribution, _gui_pauses,
										  &best_fps1[pos], 
										  &best_fps2[pos]); // default is null; only in 3D used;

				RESULT temp_result;
				temp_result.load = load;
				// retrieve latency as final result of the vNOC simulator;
				temp_result.latency = _sketch_latency;
				temp_result.buffer_multiplier = buffer_multiplier;
				// with a single injection load the latency will be stored at
				// best_fps[pos].results[0].latency
				// if a load sweep is done, we'll have results for 10% at results[0],
				// 20% at results[1], 30% at results[2], ...
				best_fps1[pos].results[buffer_multiplier-1].push_back( temp_result);

				// prepare for the next simulation if a load sweep is being done;
				load = load + 10;
			} while ( _load_sweep && load <= 100);

			// prepare for the next round if a buffer size sweep is being done also;
			buffer_multiplier ++;

		} while ( _buffer_sweep && buffer_multiplier <= 5);

	}

	return result;
}

////////////////////////////////////////////////////////////////////////////////
//
// 3D floorplanning is done on the second layer considering the floorplanning
// already done on the first layer;
//
////////////////////////////////////////////////////////////////////////////////

bool SFRA::search_n_fps_floorplans_3D_version2(
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2) 
{

	bool result = true;


	//###########################################
	//# Create top level task/application graph #
	//###########################################

	B_Tree *fp_p;

	// (1) load the original floorplan testcase;
	fp_p = new B_Tree( _alpha);
	fp_p->set_sfra_host( this); // place a copy of this host in the fp object;
	fp_p->read( _inputfile, _fp_scale);

	if ( _verbose) {
		fp_p->show_modules(); // debug;
	}
	fp_p->init();

	// Note: in a one time action only create the _application_graph;
	if ( _application_graph.nodes_count() <= 0) {
		printf("\nCreate the application_graph...");
		_application_graph.create_nodes_and_arcs( fp_p);
		_application_graph.set_host( this);
		if ( _verbose) {
			_application_graph.print_application_graph(); // debug;
		}
		printf("\nDone create application_graph.\n");
	}


	//###########################################
	//# Partition and create 2 sub-floorplans   #
	//###########################################
	
	vector<int> partition;
	int appl_nodes_count = _application_graph.nodes_count();
	bool use_hmetis = true;
	if ( use_hmetis == true) {
		// run hMetis partitioner;
		_application_graph.run_partitioning( 2); // num_partitions = 2;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			partition.push_back( _application_graph.get_node_partition( i));
		}
	}
	else {
		for ( int i = 0; i < appl_nodes_count; i ++) {
			partition.push_back( -1);
		}
		// sort IP/cores after their area and assign them into partitions
		// alternatively;
		vector<PAIR_TWO> nodes_areas;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			nodes_areas.push_back( 
				PAIR_TWO(i, _application_graph.get_node_area( i)));
		}
		sort( nodes_areas.begin(), nodes_areas.end()); // small->big;
		for ( int i = 0; i < appl_nodes_count; i ++) {
			int id = nodes_areas[i].id();
			if ( (id % 2) > 0) {
				partition[ id] = 1;
				//printf("%d(%d): 1 ",id,_application_graph.get_node_area( id));
			} else {
				partition[ id] = 0;
				//printf("%d(%d): 1 ",id,_application_graph.get_node_area( id));
			}
		}
	}
	

	// () construct the magic mappings between top-level floorplan
	// modules ids and the ids inside the subfloorplans; need these mappings
	// to be able to jump from ones to the others for assignment and then for 
	// vNOC simulation purposes;
	// let's say that in the top level we have 10 and subfloorplan 1 has 5 
	// cores out of the 10; 
	// _magic_topid_to_subid = {0 -1 -1 1 2 -1 -1 -1 3 4}; top-level node
	// id 9 corresponds to subfloorplan node id 4;
	// _magic_subid_to_topid = {0 3 4 8 9};
	int counter1_i = 0, counter2_i = 0;
	for ( int i = 0; i < appl_nodes_count; i ++) {
		if ( partition[i] == 0) {
			_magic_topid_to_subid.push_back( counter1_i);
			_magic_subid1_to_topid.push_back( i);
			counter1_i ++;
		} else { // partition[i] == 1;
			_magic_topid_to_subid.push_back( counter2_i);
			_magic_subid2_to_topid.push_back( i);
			counter2_i ++;
		}				
	}


	//#########################################
	//# Try a 'n_fps' number of subfloorplans #
	//#########################################

	search_n_fps_for_subfloorplans_layers_12(
		fp_p, // toplevel floorplan;
		partition,
		best_fps1, best_fps2);


	// () clean-up;
	delete fp_p;

	return result;
}

void SFRA::search_n_fps_for_subfloorplans_layers_12(
	FPlan *fp_toplevel, // top-level floorplan application;
	vector<int> partition,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
	vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2)
{
	// () run the floorplanner n_fps times and record n_best ones;
	// actually it may be more than n_fps in order to seek aspect_ratio
	// closer to 1;
	vector<ONE_OF_THE_BEST_FLOOR_PLANS>::iterator pos_Iterator1, pos_Iterator2;
	ONE_OF_THE_BEST_FLOOR_PLANS temp_best_floor_plan1, temp_best_floor_plan2;
	Module_Info temp_module;
	double fp_aspect_ratio = 1.0;
	int num_rejected_fps = 0;
	int THRESHOLD_TRIALS_COUNT_PER_ATTEMPT = 6;

	// loop _n_fps times and run the BTree floorplanner; record the best ones;
	for ( int i = 1; i <= _n_fps; i++) {


		// (0) create the subfloorplans on layers 1 and 2;

		B_Tree *fp_p1;
		fp_p1 = new B_Tree( _alpha);
		fp_p1->set_sfra_host( this);

		B_Tree *fp_p2;
		fp_p2 = new B_Tree( _alpha);
		fp_p2->set_sfra_host( this);


		//###############################
		//# Annealing of subfloorplan 1 #
		//###############################

		printf("\n\n\n\n\n\nSubFloorplan 1 annealing attempt number: %d\n\n", i);

		// (1.a)
		fp_p1->create_subfloorplan( 0, fp_toplevel, partition);

		int modules_N_1 = fp_p1->modules_N; // modules count of subfloorplan 1;
		if ( _verbose) {
			fp_p1->show_modules(); // debug;
		}
		fp_p1->init();

		// (2.a) run floorplaning;
		SA_FLOORPLANING sa_floorplanning1( fp_p1, _times, _local, _term_temp);
		sa_floorplanning1.set_avg_ratio( _avg_ratio);
		sa_floorplanning1.set_lambda( _lambda);
		sa_floorplanning1.set_alpha( _alpha);
		sa_floorplanning1.set_fp_scale( _fp_scale);
		sa_floorplanning1.set_verbose(_verbose); // print detailed info?
		sa_floorplanning1.run_SA_Floorplaning(); // run floorplanner;

		// (3.a)
		fp_p1->list_information();
		fp_p1->show_tree();

		//###############################
		//# Annealing of subfloorplan 2 #
		//###############################

		printf("\n\n\n\n\n\nSubFloorplan 2 annealing attempt number: %d\n\n", i);

		// (1.b)
		fp_p2->create_subfloorplan_layer2( fp_toplevel, fp_p1, partition);

		int modules_N_2 = fp_p2->modules_N; // modules count of subfloorplan 2;
		if ( _verbose) {
			fp_p2->show_modules(); // debug;
		}
		fp_p2->init();

		// (2.b) run floorplaning;
		SA_FLOORPLANING sa_floorplanning2( fp_p2, _times, _local, _term_temp);
		sa_floorplanning2.set_avg_ratio( _avg_ratio);
		sa_floorplanning2.set_lambda( _lambda);
		sa_floorplanning2.set_alpha( _alpha);
		sa_floorplanning2.set_fp_scale( _fp_scale);
		sa_floorplanning2.set_verbose(_verbose); // print detailed info?
		sa_floorplanning2.run_SA_Floorplaning(); // run floorplanner;

		// (3.b)
		fp_p2->list_information();
		fp_p2->show_tree();




		// (4) compute aspect ratio of fp_p1;
		fp_aspect_ratio = ( fp_p1->Height >= fp_p1->Width) ?
			( fp_p1->Height / fp_p1->Width) : ( fp_p1->Width / fp_p1->Height);

		// if the aspect ratio is higher than 1.3 reject this subfloorplan;
		if ( num_rejected_fps <= THRESHOLD_TRIALS_COUNT_PER_ATTEMPT && 
			fp_aspect_ratio > 1.3) {
			num_rejected_fps ++;
			if ( num_rejected_fps == _n_fps) {
				// I do this because for hp testcase because cores are
				// thin and tall rects, the floorplanner cannot find
				// floorplans with aspect ratio closed to 1; hence, I 
				// relax the wirelength requirement;
				_alpha = fmin( _alpha + 0.5, 1);
			}
			// cancel this attempt and do not count it as part of _n_fps;
			// however, this should not be done too many (infinite) times;
			i --;
			delete fp_p1;
			delete fp_p2;
			continue;
		}
		// reset num_rejected_fps so that during next "i" iteration we'll
		// explore more floorplans until we get one with good aspect ratio;
		num_rejected_fps = 0;
		

		//#################################################
		//# Verify if the resulting FP is one of the best #
		//#################################################

		bool this_fp_is_better = false; // should go into bests list;
		double worst_crit = 0;
		int pos_best, pos_worst, erase_pos;
		double aspect_ratio = 1.0;

		
		// start filling the list...
		if ( best_fps1.size() < _n_best) {
			this_fp_is_better = true; // first three fp's go into list anyway;
			pos_best = -1; // signals any position, just use push_back;
		} else {
			// if list has already a number of n_bests, find the 
			// "worst of the best" to be potentially replaced with the new one;
			for ( int pos = 0; pos < best_fps1.size(); pos++) {
				aspect_ratio = 
					( best_fps1[pos].Height >= best_fps1[pos].Width) ?
					( best_fps1[pos].Height / best_fps1[pos].Width) :
					( best_fps1[pos].Width / best_fps1[pos].Height);
				switch ( _fp_criteria) {
				case 'W':
					if (aspect_ratio > 1.3 || best_fps1[pos].WireLength > worst_crit) {
						worst_crit = best_fps1[pos].WireLength;
						pos_worst = pos;
					}
					break;
				case 'A':
					if (aspect_ratio > 1.3 || best_fps1[pos].Area > worst_crit) {
						worst_crit = best_fps1[pos].Area;
						pos_worst = pos;
					}
					break;
				default:
					assert(false);
				}
			}
			// compute also the aspect ratio of the worst floorplan in 
			// current bests list;
			aspect_ratio = 
				( best_fps1[pos_worst].Height >= best_fps1[pos_worst].Width) ?
				( best_fps1[pos_worst].Height / best_fps1[pos_worst].Width) :
				( best_fps1[pos_worst].Width / best_fps1[pos_worst].Height);
			// after scaning the whole bests list, if the actual is better than
			// the "worst of the best", assign it to the list;
			switch ( _fp_criteria) {
			case 'W':
				if ( (fp_p1->getWireLength() < best_fps1[pos_worst].WireLength) ||
					(fp_p1->getWireLength() < 1.5*best_fps1[pos_worst].WireLength &&
					fp_aspect_ratio < aspect_ratio) ) {
					pos_best = pos_worst;
				 	this_fp_is_better = true;
				}
				break;
			case 'A':
				if ( (fp_p1->getArea() < best_fps1[pos_worst].Area) ||
					(fp_p1->getArea() < 1.5*best_fps1[pos_worst].Area &&
					fp_aspect_ratio < aspect_ratio)) {
					pos_best = pos_worst;
					this_fp_is_better = true;
				}
				break;
			}
		}


		if ( this_fp_is_better) {
			printf("\nAttempts number %d go to the list of best subfloorplans...\n",i);

			// (fp_p1) copy all the info from subfloorplan 1 to a temporary container;
			temp_best_floor_plan1.attempt_n = i;
			temp_best_floor_plan1.Width = fp_p1->Width;
			temp_best_floor_plan1.Height = fp_p1->Height;
			temp_best_floor_plan1.Area = fp_p1->getArea();
			temp_best_floor_plan1.WireLength = fp_p1->getWireLength();
			temp_best_floor_plan1.modules_N = modules_N_1;
			for ( int id = 0; id < modules_N_1; id++) {
				// copy all info of each module to a temporary module;
				temp_module.x = fp_p1->modules_info[id].x;
				temp_module.y = fp_p1->modules_info[id].y;
				temp_module.rx = fp_p1->modules_info[id].rx;
				temp_module.ry = fp_p1->modules_info[id].ry;
				temp_module.rotate = fp_p1->modules_info[id].rotate;
				temp_module.flip = fp_p1->modules_info[id].flip;
				// push that temp. module to the vector inside the temp. best fp;
				temp_best_floor_plan1.modules_info.push_back( temp_module);
			}
			// add this temp. to the vector of best floorplans;
			if ( pos_best == -1)
				best_fps1.push_back( temp_best_floor_plan1);
			else {
				printf("\nRemoving attempt number %d from the bests list 1...\n",
					best_fps1[pos_worst].attempt_n);
				pos_Iterator1 = best_fps1.begin(); // erase() requires an iterator;
				erase_pos = 0;
				while ( erase_pos < pos_worst) {
					erase_pos ++;
					pos_Iterator1 ++;
				}
				best_fps1.erase( pos_Iterator1);
				best_fps1.push_back( temp_best_floor_plan1);
			}

			// (fp_p2) copy all the info from subfloorplan 2 to a temporary container;
			temp_best_floor_plan2.attempt_n = i;
			temp_best_floor_plan2.Width = fp_p2->Width;
			temp_best_floor_plan2.Height = fp_p2->Height;
			temp_best_floor_plan2.Area = fp_p2->getArea();
			temp_best_floor_plan2.WireLength = fp_p2->getWireLength();
			temp_best_floor_plan2.modules_N = modules_N_2;
			for ( int id = 0; id < modules_N_2; id++) {
				// copy all info of each module to a temporary module;
				temp_module.x = fp_p2->modules_info[id].x;
				temp_module.y = fp_p2->modules_info[id].y;
				temp_module.rx = fp_p2->modules_info[id].rx;
				temp_module.ry = fp_p2->modules_info[id].ry;
				temp_module.rotate = fp_p2->modules_info[id].rotate;
				temp_module.flip = fp_p2->modules_info[id].flip;
				// push that temp. module to the vector inside the temp. best fp;
				temp_best_floor_plan2.modules_info.push_back( temp_module);
			}
			// add this temp. to the vector of best floorplans;
			if ( pos_best == -1)
				best_fps2.push_back( temp_best_floor_plan2);
			else {
				printf("\nRemoving attempt number %d from the bests list 2...\n",
					best_fps2[pos_worst].attempt_n);
				pos_Iterator2 = best_fps2.begin(); // erase() requires an iterator;
				erase_pos = 0;
				while ( erase_pos < pos_worst) {
					erase_pos ++;
					pos_Iterator2 ++;
				}
				best_fps2.erase( pos_Iterator2);
				best_fps2.push_back( temp_best_floor_plan2);
			}
		}


		// prepare for the next floorplan
		temp_best_floor_plan1.modules_info.clear();
		temp_best_floor_plan2.modules_info.clear();
		delete fp_p1;
		delete fp_p2;
	}
}

void SFRA::calculate_average_path_length(
	ROUTER_ASSIGNMENT *router_assignment)
{
	// () calculate the average path length among all communications;
	// should not be normaly used because it adds extra CPU time;
	// this is only for debug purposes;

	B_Tree *fp_p = new B_Tree( _alpha);
	fp_p->set_sfra_host( this); // place a copy of this host in the fp object;
	fp_p->read( _inputfile, _fp_scale);
	fp_p->init();

	pair <int, int> mesh_pos_src;
	pair <int, int> mesh_pos_des;
	int total_paths_length = 0;
	int no_commodity = 0;
	for (int i=0; i < fp_p->modules_N; i++){
		for (int j=i+1; j < fp_p->modules_N; j++){
			if ( i != j && fp_p->get_connection(i,j) > 0) {
				// get src and des addresses;
				mesh_pos_src = router_assignment->core_id_to_router_xy(i);
				mesh_pos_des = router_assignment->core_id_to_router_xy(j);
				no_commodity ++;
				total_paths_length += 
					( abs(mesh_pos_src.first - mesh_pos_des.first) +
					  abs(mesh_pos_src.second - mesh_pos_des.second) );
			}	
		}
	}

	printf("\nAvg. path length for %d communications: %.2f\n",no_commodity,
		   double(total_paths_length)/no_commodity);
	delete fp_p;
}
