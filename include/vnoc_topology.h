#ifndef _VNOC_TOPOLOGY_H_
#define _VNOC_TOPOLOGY_H_

#include "config.h"
#include "vnoc_utils.h"
#include <vector>
#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace std;

enum ROUTING_ALGORITHM { ROUTING_XY = 0, ROUTING_TXY = 1 };
enum VIRTUAL_CHANNEL_SHARING { SHARED, NOT_SHARED };
enum VC_STATE { INIT, ROUTING, VC_AB, SW_AB, SW_TR, HOME };

typedef vector<long> ADDRESS;
typedef pair<long, long> VC_TYPE;
typedef vector<unsigned long long> DATA; // data "carried" by each flit;
typedef unsigned long long DATA_ATOMIC_UNIT; // 64 bits for now; we could take it as param;

const VC_TYPE VC_NULL = VC_TYPE(-1, -1); 

#define REPORT_STATS_PERIOD 2000

#define BUFF_BOUND 100

// here are a few basic physical parameters;
#define BASE_WIRE 1000
#define TSV_VIA_DELAY 0.00372316 // 0.00372316;
#define ROUTER_AREA 621652
#define ROUTER_SIDE 788

#define WIRE_DELAY 1.0 // 0.9;
#define PIPE_DELAY 1.0 // 1.0;
#define CREDIT_DELAY 1.0 // 1.0;
#define S_ELPS 0.00000001
#define ATOM_WIDTH 64
#define ZERO 0
#define MAX_64_ 0xffffffffffffffffLL
#define CORR_EFF 0.8

#define INPUT_BUFFER_SIZE 5
#define OUTPUT_BUFFER_SIZE 5
#define VIRTUAL_CHANNEL_COUNT 3
#define FLIT_SIZE 1
#define LINK_BW 64

////////////////////////////////////////////////////////////////////////////////
//
// TOPOLOGY
//
////////////////////////////////////////////////////////////////////////////////

class TOPOLOGY 
{
	private:
		// these are set thru topology file:
		long _network_size; // k-ary;
		long _cube_size; // n-cube;
		long _input_buffer_size; // buffer size for each virtual channel;
		long _output_buffer_size; // output buffer size;
		long _vc_number; // virtual channels count per input port;
		long _flit_size; // flit size as multiple of "64 bits";
		// the bandwidth - in bits - of physical link; default is 64 bits, which
		// is the default of minimum flit dimension (ie _flit_size = 1);
		long _link_bandwidth;
		long _pipeline_stages_per_link; // number of pipeline registers for each link?
		// these are set thru command line arguments:
		string _topology_file;
		string _trace_file;
		ROUTING_ALGORITHM _routing_algo;
		double _simulation_cycles_count;
		double _warmup_cycles_count;
		// virtual channel sharing? depends on the type of routing;
		VIRTUAL_CHANNEL_SHARING _vc_sharing_mode;
		long _rng_seed; // seed for random number generator; else is set to 1;
		RANDOM_NUMBER_GENERATOR _rng;
		bool _use_gui;
		// this option should be used with "use_gui"; if set true by user, 
		// then each user will have to hit "Proceed" button to advance the
		// simulation after every printing interval; default is false;
		bool _user_step_by_step; // -user_sbs;
		double _link_length; // physical link length in um;

	public:
		TOPOLOGY( long net_size, long inp_buf, long out_buf, long vc_n,
				  long flit_size, long link_bw, double link_l,
				  long pipeline_in_link, string trace_file, long seed,
				  ROUTING_ALGORITHM routing_a, double cycles,
				  double warmup_cycles, bool use_gui);
		~TOPOLOGY() {};

		bool check_topology();
		void print_topology();
		long network_size() const { return _network_size;}
		long cube_size() const { return _cube_size; }
		long virtual_channel_number() const { return _vc_number; }
		long input_buffer_size() const { return _input_buffer_size; }
		long output_buffer_size() const { return _output_buffer_size; }
		long flit_size() const { return _flit_size; }
		long flit_size() { return _flit_size; }
		long link_bandwidth() const { return _link_bandwidth; }
		double link_length() const { return _link_length; }
		ROUTING_ALGORITHM routing_algo() const { return _routing_algo; }
		double simulation_cycles_count() const { return _simulation_cycles_count; }
		double warmup_cycles_count() const { return _warmup_cycles_count; }
		VIRTUAL_CHANNEL_SHARING vc_sharing_mode() const { return _vc_sharing_mode; }
		string trace_file() { return _trace_file; }
		long rng_seed() const { return _rng_seed; }
		RANDOM_NUMBER_GENERATOR &rng() { return _rng; }
		bool use_gui() { return _use_gui; }
		bool user_step_by_step() { return _user_step_by_step; }


		void set_network_size(long network_size) 
			{ _network_size = network_size; };
		void set_input_buffer_size(long input_buffer_size) 
			{ _input_buffer_size = input_buffer_size; };
		void set_output_buffer_size(long output_buffer_size) 
			{ _output_buffer_size = output_buffer_size; };
		void set_vc_number(long vc_number) { _vc_number = vc_number; };
		void set_flit_size(long flit_size) { _flit_size = flit_size; };
		void set_link_bandwidth(long link_bandwidth) { _link_bandwidth = link_bandwidth; };
		void set_link_length(double link_length) { _link_length = link_length; };
		void set_trace_file(string trace_file) { _trace_file = trace_file; };
		void set_rng_seed(long rng_seed) {
			_rng_seed = rng_seed;
			// now set the actual seed of the internal random gen;
			_rng.set_seed( _rng_seed);
		};
		void set_routing_algo(ROUTING_ALGORITHM routing_algo) 
			{ _routing_algo = routing_algo; };
		void set_simulation_cycles_count(long simulation_cycles_count) 
			{ _simulation_cycles_count = simulation_cycles_count; };
		void set_warmup_cycles_count(long warmup_cycles_count) 
			{ _warmup_cycles_count = warmup_cycles_count; };
		void set_use_gui(bool use_gui) { _use_gui = use_gui; };
};

#endif
