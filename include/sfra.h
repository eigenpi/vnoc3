#ifndef _SFRA_H_
#define _SFRA_H_
#include "config.h"
#include "vnoc_topology.h"
#include "fp_btree.h"
#include <utility>
#include "hmetisInterface.h"

using namespace std;


// corners of IP/cores will be "hooked up" to routers of the NoC;
enum CORNER_NAME { SW, SE, NW, NE };
// this is the type of simulated system architecture: 2D with IP's
// inflated or other ideas, and 25D whcih is the 3D architecture with
// only two layers: one IPs and one NoC - called 2.5D;
// 3D is with 3 layers with the middle layer dedicated to NoC and
// layers 1 and 3 to IP cores - in this case a partitioning is done first
// using hMetis;
enum SIM_MODE { SIMULATED_ARCH_2D, SIMULATED_ARCH_3D, SIMULATED_ARCH_25D };

////////////////////////////////////////////////////////////////////////////////
//
// PAIR_TWO
//
////////////////////////////////////////////////////////////////////////////////

class PAIR_TWO
{
 private:
	int _id;
	double _value;
 public:
	PAIR_TWO() { _id = 0; _value = 0.0; }
	PAIR_TWO(int id, double value) : _id(id), _value(value) {}
	PAIR_TWO(const PAIR_TWO &pt) : 
		_id(pt._id), _value(pt._value) {}
	~PAIR_TWO() {}

	int id() const { return _id; }
	double value() const { return _value; }
	void set_id(int id) { _id = id; }
	void set_value(double value) { _value = value; }
	void operator=(const PAIR_TWO &pt) { _id = pt._id; _value = pt._value; }
	bool operator==(const PAIR_TWO &pt) const { return (_value == pt._value); }
	bool operator!=(const PAIR_TWO &pt) const { return (_value != pt._value); }
	bool operator<(const PAIR_TWO &pt)  const { return (_value  < pt._value); }
};

////////////////////////////////////////////////////////////////////////////////
//
// ROUTERS_DISTRIBUTION
//
////////////////////////////////////////////////////////////////////////////////

class ROUTERS_DISTRIBUTION 
{
 public:
	long _nx, _ny; // mesh configuration (right now nx = ny = ary);
	long _sx, _sy; // in the future could not start at (0,0) as usual;
	double _link_length; // distance between routers;

 public:
	ROUTERS_DISTRIBUTION( long x, long y, long x0, long y0, double ll);

	long nx() { return _nx; }
	long ny() { return _ny; }
	long sx() { return _sx; }
	long sy() { return _sy; }
	double link_length() { return _link_length; }
};

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_ASSIGNMENT
//
////////////////////////////////////////////////////////////////////////////////

class ROUTER_ASSIGNMENT {

 private:
	int **_array_ids;
	CORNER_NAME **_array_corners;
	double **_array_extralinks;
	int _n_rows;
	int _n_columns;
	double _total_extralinks;

 public:
	ROUTER_ASSIGNMENT(int ary);

	void assign(int x, int y, int id, CORNER_NAME corner, double extralink);
	int assigned(int x, int y);
	CORNER_NAME corner_used(int x, int y);
	pair<int, int> core_id_to_router_xy(int id);
	CORNER_NAME corner_used_by_id(int id);
	double extralink_by_id(int id);
	void clear_assignments();

	double total_extralinks() { return _total_extralinks; }
};

////////////////////////////////////////////////////////////////////////////////
//
// ONE_OF_THE_BEST_FLOOR_PLANS
//
////////////////////////////////////////////////////////////////////////////////

class RESULT {
 public:
	// percentage of the injection load used to be assigned after simulation;
	int load;
	double latency;
	double packets_per_cycle;
	int buffer_multiplier; // the buffers size (if multiplied);
 public:
	RESULT() {}
	~RESULT() {}	
};

typedef struct vector<RESULT> RESULTS;	// we use a list as a load sweep is done;

class ONE_OF_THE_BEST_FLOOR_PLANS 
{
 public:
	int attempt_n;
	Modules_Info modules_info;
	double Width, Height;
	double Area;
	long WireLength;
	long modules_N;
	vector<RESULTS> results; // we use a list as a buffer size sweep is done;
};

////////////////////////////////////////////////////////////////////////////////
//
// APPLICATION
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// APPLICATION_NODE
//
////////////////////////////////////////////////////////////////////////////////

class APPLICATION_NODE {
 private:
	int _id;
	int _area; // area of corresponding IP/core;
	int _partition; // 0 -> will be on layer 1; 1 -> will be on layer 3;
	double _io_comm_volume; // sum of comm_volume of all arcs in or out;
	vector<long> _fanin; // id's of nodes with arcs as destination this node;
	vector<long> _fanout;	
		
 public:
	APPLICATION_NODE() { 
		_id = -1; 
		_io_comm_volume = 0.;
		_area = 1;
		_partition = -1;
	}
	APPLICATION_NODE( int id) : _id(id) { 
		_io_comm_volume = 0.;
		_area = 1;
		_partition = -1;
	}
	APPLICATION_NODE( int id, int area) : _id(id), _area(area) { 
		_io_comm_volume = 0.;
		_partition = -1;
	}
	APPLICATION_NODE(const APPLICATION_NODE &node) :
		_id(node._id), _area(node._area), _partition(node._partition),
		_io_comm_volume(node._io_comm_volume) {}
	~APPLICATION_NODE() {}
	
	int id() const { return _id; }
	void set_partition( int p) { _partition = p; }
	int partition() { return _partition; }
	void set_area( int a) { _area = a; }
	int area() { return _area; }
	void set_io_comm_volume( double v) { _io_comm_volume = v; }
	double io_comm_volume() { return _io_comm_volume; }
	void add_to_io_comm_volume(double delta) { _io_comm_volume += delta; }
	vector<long> &fanin() { return _fanin; }
	vector<long> &fanout() { return _fanout; }
	void add_fanout( long id) { // id is index of core this one fanouts to;
		_fanout.push_back( id);
	}
	void add_fanin( long id) {
		_fanin.push_back( id);
	}
};

////////////////////////////////////////////////////////////////////////////////
//
// APPLICATION_ARC
//
////////////////////////////////////////////////////////////////////////////////

class APPLICATION_ARC {
 private:
	int _id;
	long _src_id; // id of core that is source of this arc;
	long _des_id; // id of core that is destination of this arc;
	double _comm_volume;
		
 public:
	APPLICATION_ARC( int id, int src, int des, double cv) : 
		_id(id), _src_id(src), _des_id(des), _comm_volume(cv) {
	}
	~APPLICATION_ARC() {}
	
	int id() const { return _id; }
	int src_id() const { return _src_id; }
	int des_id() const { return _des_id; }
	double comm_volume() { return _comm_volume; }
	void set_comm_volume( double comm_volume) 
		{ _comm_volume = comm_volume; }
};

////////////////////////////////////////////////////////////////////////////////
//
// APPLICATION_GRAPH
//
////////////////////////////////////////////////////////////////////////////////

class SFRA;

class APPLICATION_GRAPH {
 private:
	long _nodes_count;
	long _arcs_count;
	vector<APPLICATION_NODE> _nodes; // IP/cores of the application;
	vector<APPLICATION_ARC> _arcs; // communication arcs of the application;
	double _max_nodes_comm_volume; // maximum communication volume among all nodes; 
	double _max_arcs_comm_volume;
	SFRA *_sfra; // its host;
	MetisIntfc *_hmetis_interface;

 public:
	APPLICATION_GRAPH() { _nodes_count = 0; _arcs_count = 0; }
	~APPLICATION_GRAPH() {}
	
	void set_host( SFRA *sfra) { _sfra = sfra; }
	SFRA *sfra() { return _sfra; }	
	long nodes_count() { return _nodes_count; }
	long arcs_count() { return _arcs_count; }
	double max_nodes_comm_volume() { return _max_nodes_comm_volume; }
	double max_arcs_comm_volume() { return _max_arcs_comm_volume; }
	vector<APPLICATION_NODE> &nodes() { return _nodes; }
	APPLICATION_NODE *get_node(long id) {
		//assert(id >= 0 && id < _nodes_count);	
		return &_nodes[ id]; 
	}
	APPLICATION_ARC *get_arc(long id) {
		return &_arcs[ id]; 
	}
	bool create_nodes_and_arcs( B_Tree *fp_p);
	
	void compute_max_nodes_comm_volume() {
		// this should be called once only;
		_max_nodes_comm_volume = 0.0;
		for ( long i = 0; i < _nodes_count; i++) {
			double node_comm_volume = 0.0;
			for ( long k = 0; k < _nodes[i].fanin().size(); k ++) {
				node_comm_volume += _arcs[ _nodes[i].fanin()[k] ].comm_volume();
			}
			for ( long k = 0; k < _nodes[i].fanout().size(); k ++) {
				node_comm_volume += _arcs[ _nodes[i].fanout()[k] ].comm_volume();
			}
			_nodes[i].set_io_comm_volume( node_comm_volume); // record it;
			if ( _max_nodes_comm_volume < _nodes[i].io_comm_volume()) {
				_max_nodes_comm_volume = _nodes[i].io_comm_volume();
			}
		}
	}
	void compute_max_arcs_comm_volume() {
		_max_arcs_comm_volume = 0.0;
		for ( long i = 0; i < _arcs_count; i++) {
			if ( _max_arcs_comm_volume < _arcs[i].comm_volume()) {
				_max_arcs_comm_volume = _arcs[i].comm_volume();
			}
		}
	}
	void print_application_graph();
	// hMetis related;
	void init_hmetis_interface();
	void get_hmetis_results();
	void print_graph_partitions();
	bool run_partitioning( int num_partitions = 2); // by default do bi-partitioning;
	int get_node_partition( int i) {
		//assert(i >= 0 && i < _nodes_count);
		return ( _nodes[ i].partition());
	}
	int get_node_area( int i) {
		//assert(i >= 0 && i < _nodes_count);
		return ( _nodes[ i].area());
	}
};

////////////////////////////////////////////////////////////////////////////////
//
// SFRA - means simultaneous floorplaning and router assignment;
//
////////////////////////////////////////////////////////////////////////////////

class SFRA {
 private:
	// floorplaning annealing parameters;
	int _times; // used directly as argument when SA_Floorplan() is called;
	int _local; // used directly as argument when SA_Floorplan() is called;
	float _term_temp; // used directly as argument when SA_Floorplan() is called;
	float _avg_ratio;
	float _lambda;
	float _alpha;
	float _fp_scale;

	int _modules_N; // number of cores of the floorplan for a given testcase;
	int _ary; // quantity of routers in both dimensions; total number of routers;

	// big loop optimization (rather search and retain) parameters;
	char _inputfile[256];
	int _n_fps, _n_best;
	char _fp_criteria;

	// vNOC simulation parameters;
	long _inp_buf, _out_buf;
	long _vc_n, _flit_size;
	long _link_bw, _link_l, _pipeline_in_link;
	long _seed;
	ROUTING_ALGORITHM _routing_a;
	double _cycles, _warmup_cycles;
	bool _use_gui;
	int _x_ary;
	double _extra_links_timing_factor;
	int _gui_pauses;
	int _sim_mode;
	char _test_name[256];
	int _inj_load, _skipped_packets, _max_skip_counter, _skip_counter;

	// additional parameters for various sweeps and investigations;
	bool _load_sweep;
	bool _buffer_sweep;
	bool _testcase_creation;
	float _testcase_multiplier;
	bool _use_excel;

	// _sketch_latency is a temporary storage where any call of the vNOC
	// simulator will deposit the final result; needed here because the
	// current wrapper VNOC_APP of vNOC is used via its ctor, which will 
	// destroy vNOC object inside it; so, I need to pass the result;
	double _sketch_latency;
	double _sketch_packets_per_cycle; // average;

	// application graph; this is unique no matter how many floorplans
	// are investigated;
	APPLICATION_GRAPH _application_graph;
	// sketch arrays used for 3D architecture explaoration only;
	vector<int> _magic_topid_to_subid;
	vector<int> _magic_subid1_to_topid;
	vector<int> _magic_subid2_to_topid; 

	// if _verbose is true then detailed framework run will be printed
	// by calling "print_*" functions; default is true;
	bool _verbose;

 public:

 public:
	SFRA() : _application_graph() {
		// default values;
		_times = 160;
		_local = 6;
		_avg_ratio = 30;
		_lambda = 1.3;
		_term_temp = 0.1; 
		_alpha = 0.25;
		_n_fps = 1; // default 10;
		_n_best = 1; // default 3;
		_fp_criteria = 'A';
		_inp_buf = INPUT_BUFFER_SIZE; // 5;
		_out_buf = OUTPUT_BUFFER_SIZE; // 5;
		_vc_n = VIRTUAL_CHANNEL_COUNT; // 3;
		_flit_size = FLIT_SIZE; // 1: flit is same a phit: 64 bits;
		_link_bw = LINK_BW; // 64;
		_pipeline_in_link = 0;
		_routing_a = ROUTING_XY;
		_use_gui = false;
		_x_ary = 0;
		_extra_links_timing_factor = 1;
		_gui_pauses = 1;
		_fp_scale = 1.0;
		_sim_mode = SIMULATED_ARCH_25D;
		strcpy( _test_name, "Unnamed");
		_inj_load = 100; 
		_skipped_packets = 0; 
		_max_skip_counter = 10;
		_skip_counter = 0;
		_load_sweep = false;
		_buffer_sweep = false;
		_testcase_creation = false;
		_testcase_multiplier = 1.0;
		_use_excel = false;

		_modules_N = 0;
		_ary = 0;
		_sketch_latency = 0.0;
		_sketch_packets_per_cycle = 0.0;
		// seed to be used for the C++' RNG is set randomly here; 
		// it can be overwritten via arguments by user;
		_seed = time(NULL);
		_verbose = true;
	}
	~SFRA() {}

	bool parse_command_arguments( int argc, char *argv[]);
	void print_setup_info() {
		printf("Testcase name: \"%s\"", &_test_name);
		switch ( _sim_mode) {
		case SIMULATED_ARCH_2D:
			printf("\nWorking in 2D mode.");
			break;
		case SIMULATED_ARCH_3D:
			printf("\nWorking in 3D mode.");
			break;
		case SIMULATED_ARCH_25D:
			printf("\nWorking in 2.5D mode.");
			break;
		default:
			assert(0);
		}
		printf("\nSeed: %d", _seed);
	}
	void setup_seed() {
		// use the seed that potentially was asked by user via arguments;
		srand( _seed);
	}
	// 2D, 25D;
	bool search_n_fps_floorplans(
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps);
	bool routers_assignment_and_vNOC_simulation(
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps);
	bool calculate_final_results_statistics( int argc,char **argv,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps);
	// 3D;
	bool search_n_fps_floorplans_3D(
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2);
	void search_n_fps_for_subfloorplan( B_Tree *fp_p,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps);
	bool routers_assignment_and_vNOC_simulation_3D(
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2);
 	int magic_subid1_to_topid(int i) const { 
		return _magic_subid1_to_topid[ i]; 
	};
 	int magic_subid2_to_topid(int i) const { 
		return _magic_subid2_to_topid[ i]; 
	};
	// 3D version 2;
	bool search_n_fps_floorplans_3D_version2(
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2);
	void search_n_fps_for_subfloorplans_layers_12(
		FPlan *fp_toplevel,
		vector<int> partition,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps1,
		vector <ONE_OF_THE_BEST_FLOOR_PLANS> &best_fps2);
 

	double extra_links_timing_factor() const { return _extra_links_timing_factor; }
	int sim_mode() const { return _sim_mode; }
	void set_skip_counter(int new_val) { _skip_counter = new_val; }
	int skip_counter() const { return _skip_counter; }
	int max_skip_counter() const { return _max_skip_counter; }
	bool testcase_creation() const { return _testcase_creation; }
	double testcase_multiplier() const { return _testcase_multiplier; }
	void set_sketch_latency(double val) { _sketch_latency = val; }
	double sketch_latency() const { return _sketch_latency; }
	void set_sketch_packets_per_cycle(double val) { _sketch_packets_per_cycle = val; }
	double sketch_packets_per_cycle() const { return _sketch_packets_per_cycle; }
	char *test_name() { return _test_name; }
	bool verbose() const { return _verbose; }
	void calculate_average_path_length(ROUTER_ASSIGNMENT *router_assignment);
};

////////////////////////////////////////////////////////////////////////////////
//
// MISC FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////

// here are some util function used as global functions inside vNOC simulator;

#endif
