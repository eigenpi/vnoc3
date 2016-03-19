#include "config.h"
#include "vnoc_utils.h"
#include "vnoc_topology.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// TOPOLOGY
//
////////////////////////////////////////////////////////////////////////////////

TOPOLOGY::TOPOLOGY( long net_size, long inp_buf, long out_buf, long vc_n,
					long flit_size, long link_bw, double link_l,
					long pipeline_in_link, string trace_file, long seed,
					ROUTING_ALGORITHM routing_a, double cycles,
					double warmup_cycles, bool use_gui) : _rng()
{
	_use_gui = use_gui;
	// reset NOC topology to defaults;
	_cube_size = 2;
	_vc_sharing_mode = SHARED;

	// record input parameters; they used to be set through
	// "parse_command_arguments" function; now we pass them as arguments
	// because the simulator will be called multiple times inside the 
	// optimization loop with the floorplanner?
	_network_size = net_size; // default 8;
	_input_buffer_size = inp_buf; // default 12;
	_output_buffer_size = out_buf; // default 12;
	_vc_number = vc_n; // default 3;
	_flit_size = flit_size; // default 1;
	_link_bandwidth = link_bw; // default 64;
	_link_length = link_l; // default 1000 um;
	_pipeline_stages_per_link = pipeline_in_link; // 0 => link is not pipelined;
	_trace_file = trace_file;
	_rng_seed = seed; // default 1;
	_routing_algo = routing_a; // default ROUTING_XY;
	_simulation_cycles_count = cycles; // default 100;
	_warmup_cycles_count = warmup_cycles; // default 10;
	_user_step_by_step = false;

	print_topology();
		
	// now set the actual seed of the internal random gen;
	_rng.set_seed( _rng_seed);
}

bool TOPOLOGY::check_topology()
{
	bool result = true;

	// very simple sanity checks;
	if ( _network_size < 2 || _network_size > 1024) {
		result = false;
	}
	if ( _cube_size < 2 || _cube_size > 4) {
		result = false;
	}
	if ( _input_buffer_size < 0 || _input_buffer_size > 1024) {
		result = false;
	}
	if ( _output_buffer_size < 0 || _output_buffer_size > 1024) {
		result = false;
	}
	if ( _vc_number < 1 || _vc_number > 128) {
		result = false;
	}
	if ( _flit_size < 1 || _flit_size > 128) {
		result = false;
	}
	if ( _link_bandwidth < 1 || _link_bandwidth > 128) {
		result = false;
	}
	if ( _link_length < 1.0e-6 || _link_length > 3000) {
		result = false;
	}
	if ( _pipeline_stages_per_link < 0 || _pipeline_stages_per_link > 32) {
		result = false;
	}

	if (!result) {
		printf("Error:  NOC topology check failed.\n\n");
		exit(1);
	}
	return result;
}

void TOPOLOGY::print_topology()
{
	printf("topology_file:            %s \n", _topology_file.c_str());
	printf("network_size:             %d \n", _network_size);
	printf("cube_size:                %d \n", _cube_size);
	printf("input_buffer_size:        %d \n", _input_buffer_size );
	printf("output_buffer_size:       %d \n", _output_buffer_size);
	printf("virtual_channel_number:   %d \n", _vc_number);
	printf("flit_size:                %d \n", _flit_size);
	printf("link_bandwidth:           %d \n", _link_bandwidth);
	printf("link_length [um]:         %e \n", _link_length);
	printf("pipeline_stages_per_link: %d \n", _pipeline_stages_per_link);
	printf("\n");
	printf("trace_file:               %s \n", _trace_file.c_str());
	printf("seed:                     %ld \n", _rng_seed);
	if ( _routing_algo == ROUTING_XY) {
		printf("routing_algo:             %s \n", "XY");
	} else if ( _routing_algo == ROUTING_TXY) {
		printf("routing_algo:             %s \n", "TXY");
	}
	printf("simulation_cycles_count:  %.2f \n", _simulation_cycles_count);
	printf("warmup_cycles_count:      %.2f \n", _warmup_cycles_count);
	printf("\n");
}
