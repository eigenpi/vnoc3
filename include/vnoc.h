#ifndef _VNOC_H_
#define _VNOC_H_
#include "config.h"
#include "vnoc_topology.h"
#include <assert.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <map>
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>

using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// "vnoc" means versatile network on chip simulator;
//
////////////////////////////////////////////////////////////////////////////////

class EVENT;
class VNOC;
class ROUTER;
class EVENT_QUEUE;
class GUI_GRAPHICS;
class SFRA;

////////////////////////////////////////////////////////////////////////////////
//
// FLIT
//
////////////////////////////////////////////////////////////////////////////////

class FLIT {
	public:
		enum FLIT_TYPE { HEADER, BODY, TAIL };
	private:
		long _id;
		FLIT_TYPE _type;
		double _start_time;
		double _finish_time;
		ADDRESS _src_addr;
		ADDRESS _des_addr;
		DATA _data; // vector<unsigned long long>;

	public:
		FLIT() : _id(0), _type(HEADER), _src_addr(0), _des_addr(0), 
			_start_time(0), _finish_time(0), _data() {}
		FLIT( long id, FLIT_TYPE type, ADDRESS &src, ADDRESS &des,
			double start_time, const DATA &data) : 
			_id(0), _type(type), 
			_src_addr(src), _des_addr(des), 
			_start_time(start_time), _finish_time(0), 
			_data(data) {}
 		FLIT( const FLIT &f) : _id(f.id()), _type(f.type()),
			_src_addr(f.src_addr()), _des_addr(f.des_addr()),
			_start_time(f.start_time()), _finish_time(f.finish_time()), 
			_data(f.data()) {}
		~FLIT() {}

		int id() const { return _id; }
		void set_id(int id) { _id = id; }
		FLIT_TYPE type() const { return _type; }
		double start_time() const { return _start_time; }
		double finish_time() const { return _finish_time; }
		ADDRESS src_addr() const { return _src_addr; }
		ADDRESS des_addr() const { return _des_addr; }
		DATA &data() { return _data; }
		const DATA &data() const { return _data; }
};

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_INPUT
//
////////////////////////////////////////////////////////////////////////////////

class ROUTER_INPUT {
	private:
		ROUTER *_router; // its owner;
		// input buffers: <physical port index<vc index<buffer>>>
		vector<vector<vector<FLIT> > > _input_buff;
		// state of each input vc;
		vector<vector<VC_STATE> > _vc_state; // INIT, ROUTING, VC_AB, SW_AB, SW_TR, HOME
		// candidate routing vcs; this stores the routing matrix/mapping
		// between inputs and outputs;
		vector<vector<vector<VC_TYPE> > > _routing; // pair<long, long>
		// the selected routing vc;
		vector<vector<VC_TYPE> > _selected_routing;
		// this is a flag record that the buffer of injection is full;
		// used to control the packet injection and decrease mem usage;
		bool _injection_buff_full;

	public:
		ROUTER_INPUT(long physical_ports_count, long vc_count);
		ROUTER_INPUT();
		~ROUTER_INPUT() {}

	public:
		// _input_buff;
		vector<vector<vector<FLIT> > > &input_buff() { return _input_buff; }
		const vector<vector<vector<FLIT> > > &input_buff() const 
			{ return _input_buff; }
		vector<FLIT> &input_buff(long i, long j) { return _input_buff[i][j]; } 
		const vector<FLIT> &input_buff(long i, long j) const 
			{ return _input_buff[i][j]; }
		//_vc_state;
		vector<vector<VC_STATE> > &vc_state() { return _vc_state; }
		const vector<vector<VC_STATE> > &vc_state() const { return _vc_state; }
		VC_STATE vc_state(long i, long j) { return _vc_state[i][j]; }
		const VC_STATE vc_state(long i, long j) const { return _vc_state[i][j]; }
		void vc_state_update(long i, long j, VC_STATE state)
			{ _vc_state[i][j] = state;}
		// _routing;
		vector<vector<vector<VC_TYPE> > > &routing() { return _routing; }
		const vector<vector<vector<VC_TYPE> > > &routing() const 
			{ return _routing; }
		vector<VC_TYPE> &routing(long i, long j) { return _routing[i][j];}
		const vector<VC_TYPE> &routing(long i, long j) const 
			{ return _routing[i][j]; }
		void add_routing(long i, long j, VC_TYPE t) { _routing[i][j].push_back(t); }
		void clear_routing(long i, long j) { _routing[i][j].clear(); }
		// _selected_routing;
		vector<vector<VC_TYPE> > &selected_routing() { return _selected_routing; }
		const vector<vector<VC_TYPE> > & selected_routing() const 
			{ return _selected_routing; }
		VC_TYPE &selected_routing(long i, long j) { return _selected_routing[i][j]; }
		const VC_TYPE &selected_routing(long i, long j) const 
			{ return _selected_routing[i][j]; }
		void assign_selected_routing(long i, long j, VC_TYPE c) 
			{ _selected_routing[i][j] = c; }
		void clear_selected_routing(long i, long j) 
			{ _selected_routing[i][j] = VC_NULL; }

		// utils;
		void add_flit(long i, long j, const FLIT &flit) 
			{ _input_buff[i][j].push_back(flit); }
		void remove_flit(long i, long j)
			{ _input_buff[i][j].erase(_input_buff[i][j].begin()); }
		FLIT &get_flit(long i, long j) {
			assert( _input_buff[i][j].size() > 0);
			return ( _input_buff[i][j][0]); 
		}
		FLIT &get_flit(long i, long j, long k) {
			assert( _input_buff[i][j].size() > k);
			return ( _input_buff[i][j][k]); 
		}

		void set_injection_buff_full() { _injection_buff_full = true; }
		void clear_injection_buff_full() { _injection_buff_full = false; }
		bool injection_buff_full() const {return _injection_buff_full; }
};

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_OUTPUT
//
////////////////////////////////////////////////////////////////////////////////

class ROUTER_OUTPUT {
	public:
		enum VC_USAGE { USED, FREE };
	private:
		ROUTER *_router; // its owner;
		// used for input of next router;
		long _buffer_size_next_r;
		vector<vector<long> > _counter_next_r;
		vector<vector<VC_STATE> > _flit_state; // INIT, ROUTING, VC_AB, SW_AB, SW_TR, HOME;
		// assigned for the input
		vector<vector<VC_TYPE> > _assigned_to; // pair<long, long>
		vector<vector<VC_USAGE> > _vc_usage; // USED FREE
		vector<vector<FLIT> > _out_buffer; // actual local output buffers;
		vector<vector<VC_TYPE> > _out_addr; // output address;
		vector<long> _local_counter; // one for each output port;

	public:
		ROUTER_OUTPUT(long i, long j, long c, long d);
		ROUTER_OUTPUT();
		~ROUTER_OUTPUT() {}

	public:
		long buffer_size_next_r() const { return _buffer_size_next_r; }
		vector<vector<long> > &counter_next_r() { return _counter_next_r; }
		long counter_next_r(long i, long j) const { return _counter_next_r[i][j]; }
		void counter_next_r_inc(long i, long j) { _counter_next_r[i][j]++; }
		void counter_next_r_dec(long i, long j) { _counter_next_r[i][j]--; }

		VC_STATE flit_state(long i) const { return _flit_state[i][0]; }

		const VC_TYPE &assigned_to(long i, long j) const 
			{ return _assigned_to[i][j]; }
		void acquire(long i, long j, VC_TYPE pair) 
			{ _vc_usage[i][j] = USED; _assigned_to[i][j] = pair; }
		void release(long i, long j) 
			{ _vc_usage[i][j] = FREE; _assigned_to[i][j] = VC_NULL; }
		vector<vector<VC_USAGE> > &vc_usage() { return _vc_usage; }
		VC_USAGE vc_usage(long i, long j) { return _vc_usage[i][j]; }
		VC_USAGE vc_usage(long i, long j) const { return _vc_usage[i][j]; }
		
		void add_flit(long i, const FLIT &flit)
			{ _out_buffer[i].push_back(flit); _local_counter[i]--; }
		void remove_flit(long i);
		FLIT &get_flit(long i) {
			assert( _out_buffer[i].size() > 0);
		   	return _out_buffer[i][0];
		}
		vector<FLIT> &out_buffer(long i) { return _out_buffer[i]; }
		const vector<FLIT> &out_buffer(long i) const 
			{ return _out_buffer[i]; }

		vector<vector<VC_TYPE> > out_addr() { return _out_addr; }
		const vector<vector<VC_TYPE> > out_addr() const { return _out_addr; }
		vector<VC_TYPE> out_addr(long i) { return _out_addr[i];}
		const vector<VC_TYPE> out_addr(long i) const { return _out_addr[i]; }
		VC_TYPE get_addr(long i) { return _out_addr[i][0];}
		void remove_addr(long i) { 
			assert( _out_addr[i].size() > 0); 
			_out_addr[i].erase( _out_addr[i].begin());
		}
		void add_addr(long i, VC_TYPE b) { _out_addr[i].push_back(b); }

		vector<long> &local_counter() { return _local_counter; }
		long local_counter(long i) { return _local_counter[i]; }
		long local_counter(long i) const { return _local_counter[i]; }
		void local_counter_inc(long i) { _local_counter[i]++; }
		void local_counter_dec(long i) { _local_counter[i]--; }
};

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER
//
////////////////////////////////////////////////////////////////////////////////

class ROUTER {
	private:
		VNOC *_vnoc; // its owner, which has EVENT_QUEUE *_event_queue as well;
		ADDRESS _address;
		long _id;
		ROUTER_INPUT _input; // input buffer module;
		ROUTER_OUTPUT _output; // output buffer module;
		// initial random number
		DATA _init_data;
		long _ary_size;
		long _flit_size; // how many groups of "64 bits";
		long _physical_ports_count;
		long _vc_number;
		long _buffer_size; // of each virtual channel;
		long _out_buffer_size; // of each vc out buffer size;
		// accumulated total "propagation" delay of all flits with destination
		// (ie consumed) by this router;
		double _total_delay;
		ROUTING_ALGORITHM _routing_algo; // routing algorithm used;
		double _local_injection_time; // used for the next packet injection time;
		long _packet_counter;
		ifstream *_local_injection_file; // input trace file;
		// these two are required in the implementation of the 2.5D
		// framework;
		// "extra_length" is the additional wire length needed to connect 
		// the IP/core to this router; this is required because it introduces 
		// additional delay;
		double _link_length;
		double _extra_length;
		ROUTER_ASSIGNMENT *_router_assignment;
	public:
		int _assigned_core;

	public:
		ROUTER( long physical_ports_count, long vc_number, long buffer_size, 
				long out_buffer_size, const ADDRESS &address, long ary_size, 
				long flit_size, VNOC *owner_vnoc, long id,
				ROUTER_ASSIGNMENT *router_assignment, double link_length);
		ROUTER();
		~ROUTER() {}

		void set_id(long id) { _id = id; }
		long id() { return _id; }
		void set_owner(VNOC *vnoc) { _vnoc = vnoc; }
		const vector<long> &address() const { return _address; }
		ROUTER_INPUT &input() { return _input; }
		ROUTER_OUTPUT &output() { return _output; }	
		long physical_ports_count() { return _physical_ports_count; }
		long vc_number() { return _vc_number; }
		long buffer_size() { return _buffer_size; }
		double extra_length() { return _extra_length; }

		// calculate the  accumulated total "propagation" delay of all flits;	
		void update_delay(double delta) { _total_delay += delta; }
		double total_delay() const { return _total_delay; }
		// retrieve packet from trace file associated with this;
		void receive_packet_from_trace_file();
		void inject_packet( long flit_id, ADDRESS &sor_addr, ADDRESS &des_addr,
			double time, long packet_size);
		// flit and credit utils;
		void receive_flit_from_upstream(long port_id, long vc_id, FLIT &flit);
		void consume_flit(double t, const FLIT &flit);
		void receive_credit(long i, long j);

		// run simulation of this router;
		void simulate_one_router();
		void routing_decision_stage();
		VC_TYPE vc_selection(long i, long j);
		void vc_arbitration_stage();
		void sw_arbitration_stage();
		void send_flit_to_out_buffer();
		void send_flit_via_physical_link(long i);
		void send_flits_via_physical_link();
		void sanity_check() const;

		ifstream &local_injection_file() { return *_local_injection_file; }
		void init_local_injection_file();
		void close_injection_file() { // needed to avoid memory leaks;
			_local_injection_file->close();
			delete _local_injection_file;
		}

		void call_current_routing_algorithm(
			const ADDRESS &des_t, const ADDRESS &sor_t,
			long s_ph, long s_vc);
};

////////////////////////////////////////////////////////////////////////////////
//
// LINK
//
////////////////////////////////////////////////////////////////////////////////

class LINK {
 private:
	int _id;
	int _x;
	int _y;
	
 public:
 	LINK( int x, int y, int z) : _x(x), _y(y), _id(-1) {}
	~LINK() {}
	
	int id() const { return _id; }
	void set_id(int id) { _id = id; }
};

////////////////////////////////////////////////////////////////////////////////
//
// VNOC
//
////////////////////////////////////////////////////////////////////////////////

class VNOC {
	private:
		long _ary_size;
		long _cube_size;
		long _routers_count;
		long _packets_count;
		ifstream _input_file_st;
		ROUTER_ASSIGNMENT *_router_assignment;
		// _latency is the final result of running the vNOC simulator; introduced
		// in order to be able to read it from other classes, when vNOC is used 
		// multiple times;
		double _latency;
		// if _verbose is true then detailed debug info will be printed;
		bool _verbose;
		
		// here are the delay numbers extrapolated from the original
		// numbers from popnet that were derived for a link of 1mm and
		// technology of 100nm;
		double _h_wire_delay;
		double _h_pipe_delay;
		double _h_credit_delay;

	public:
		// _routers was made public to be accessed by the gui;
		vector<ROUTER> _routers;
		TOPOLOGY *_topology;
		EVENT_QUEUE *_event_queue;
		GUI_GRAPHICS *_gui;
		SFRA *_sfra;

	public:
		VNOC( TOPOLOGY *topology, EVENT_QUEUE *event_queue,
			ROUTER_ASSIGNMENT *router_assignment, bool verbose=true);
		~VNOC() {
			// all files must be closed, or memory leaks can occur;
			_input_file_st.close();
			for ( int i = 0; i < _routers.size(); i++) {
				_routers[i].close_injection_file();
			}
		}

		TOPOLOGY *topology() const { return _topology; }
		EVENT_QUEUE *event_queue() { return _event_queue; }
		GUI_GRAPHICS *gui() { return _gui; };
		void set_gui(GUI_GRAPHICS *gui) { _gui = gui; }
		SFRA *sfra() { return _sfra; }
		void set_sfra_host(SFRA *sfra) { _sfra = sfra; }

		vector<ROUTER> &routers() { return _routers; }
		const vector<ROUTER> &routers() const { return _routers; }
		ROUTER &router(const ADDRESS &a);
		const ROUTER &router(const ADDRESS &a) const;
		ROUTER *router( long id) {
			assert( id >= 0 && id < _routers_count);
			return ( &_routers[ id]);
		}

		bool check_address(const ADDRESS &a) const;
		long ary_size() const { return _ary_size; }
		long cube_size() const { return _cube_size; }
		long routers_count() { return _routers_count; }
		long packets_count() { return _packets_count; }
		long packets_count() const { return _packets_count; }
		void packets_count_incr() { _packets_count ++;}	
		double latency() const { return _latency; }
		bool verbose() const { return _verbose; }

		bool receive_EVENT_PE();
		bool receive_EVENT_ROUTER( EVENT this_event);
		bool receive_EVENT_LINK( EVENT this_event);
		bool receive_EVENT_CREDIT( EVENT this_event);

		bool run_simulation();
		void check_simulation();
		void print_simulation_results();
		void print_network_routers();
		void init_input_file();

		// delay calculations; used for sfra purposes;
		double Elmore_delay(double base_delay, double base_length, double actual_length);
		double Linear_delay( double base_delay, double base_length, double actual_length);
		double Via_delay(double base_delay, double layers);
		void compute_heterogeneous_t_clock( double max_link_length,
			bool square_proportionality = false); // linear not square by default;
		double h_wire_delay() { return _h_wire_delay; }
		double h_pipe_delay() { return _h_pipe_delay; }
		double h_credit_delay() { return _h_credit_delay; }
};

#endif

