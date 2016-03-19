#include "config.h"
#include <limits.h>
#include "sfra.h"
#include "vnoc.h"
#include "vnoc_event.h"
#include <math.h>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <algorithm>

using namespace std;


// I should encapsulate this one inside _vnoc;
unsigned long VC_MASK[10] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512};


////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_INPUT
//
////////////////////////////////////////////////////////////////////////////////

ROUTER_INPUT::ROUTER_INPUT():
	_input_buff(),
	_vc_state(),
	_routing(),
	_selected_routing(),
	_injection_buff_full(false)
{
}

ROUTER_INPUT::ROUTER_INPUT(long physical_ports_count, long vc_count):
	_input_buff(),
	_vc_state(),
	_routing(),
	_selected_routing(),
	_injection_buff_full(false)
{
	long i = 0;
	_input_buff.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		_input_buff[i].resize( vc_count);
	}
	_vc_state.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		_vc_state[i].resize( vc_count, INIT);
	}
	_routing.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		_routing[i].resize( vc_count);
	}
	_selected_routing.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		_selected_routing[i].resize(vc_count, VC_NULL);
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER_OUTPUT
//
////////////////////////////////////////////////////////////////////////////////

ROUTER_OUTPUT::ROUTER_OUTPUT():
	_buffer_size_next_r(),
	_counter_next_r(),
	_flit_state(),
	_assigned_to(),
	_vc_usage(), // USED, FREE;
	_out_buffer(),
	_out_addr(),
	_local_counter()
{
}
//a: phy size. b: vir number. c: input buffer size. d: output buffer size
ROUTER_OUTPUT::ROUTER_OUTPUT(long physical_ports_count, long vc_count,
	long input_buffer_size, long output_buffer_size):
	_buffer_size_next_r( input_buffer_size),
	_counter_next_r(),
	_flit_state(),
	_assigned_to(),
	_vc_usage(),
	_out_buffer(),
	_out_addr(),
	_local_counter()
{
	long i = 0;
	_counter_next_r.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		// initialize the counter at any output port - which maintains
		// the number of available space of input buffer of down-stream/next
		// router that has vc_count virtual channels - with input_buffer_size;
		_counter_next_r[i].resize( vc_count, input_buffer_size);
	}
	_assigned_to.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		// innitially none of the output ports is assigned 
		// to any input port;
		_assigned_to[i].resize( vc_count, VC_NULL);
	}
	_vc_usage.resize( physical_ports_count);
	for ( i = 0; i < physical_ports_count; i++) {
		_vc_usage[i].resize( vc_count, ROUTER_OUTPUT::FREE);
	}
	_out_buffer.resize( physical_ports_count);
	_flit_state.resize( physical_ports_count);
	_out_addr.resize( physical_ports_count);
	// each of the output ports has initially output_buffer_size 
	// space available;
	_local_counter.resize( physical_ports_count, output_buffer_size);
}

void ROUTER_OUTPUT::remove_flit( long i)
{ 
	_out_buffer[i].erase( _out_buffer[i].begin());
	if ( _flit_state[i].size() > 0) { // this fixed a bug in the original vnoc;
		_flit_state[i].erase( _flit_state[i].begin() );
	}
	_local_counter[i]++; 
}

////////////////////////////////////////////////////////////////////////////////
//
// ROUTER
//
////////////////////////////////////////////////////////////////////////////////

ROUTER::ROUTER () :
	_id(0),
	_vnoc(0),
	_address(),
	_input(),
	_output(),
	_ary_size(),
	_flit_size(),
	_physical_ports_count(),
	_vc_number(),
	_buffer_size(),
	_out_buffer_size(),
	_total_delay(),
	_routing_algo( ROUTING_XY), // by default we do XY;
	_local_injection_time(),
	_packet_counter(0),
	_local_injection_file()
{
}

ROUTER::ROUTER( long physical_ports_count, long vc_number, long buffer_size,
	long out_buffer_size, const ADDRESS &address, long ary_size,
	long flit_size, VNOC *owner_vnoc, long id, 
	ROUTER_ASSIGNMENT *router_assignment, double link_length):
	_id(id),
	_address(address),
	_input(physical_ports_count, vc_number),
	_output(physical_ports_count, vc_number, buffer_size, out_buffer_size),
	_ary_size(ary_size),
	_flit_size(flit_size),
	_physical_ports_count(physical_ports_count),
	_vc_number(vc_number),
	_buffer_size(buffer_size),
	_out_buffer_size(out_buffer_size),
	_total_delay(0),
	_local_injection_time(),
	_packet_counter(0),
	_local_injection_file()
{
	_vnoc = owner_vnoc;

	_router_assignment = router_assignment;
	_link_length = link_length;
	_assigned_core = _router_assignment->assigned(_address[0],_address[1]);
	_extra_length = _router_assignment->extralink_by_id( _assigned_core) *
		BTREE_TO_VNOC_SCALE_MULTIPLIER;

	_init_data.resize( flit_size);
	for (long i = 0; i < flit_size; i++) {
		_init_data[i] = _vnoc->topology()->rng().flat_ull(0, ULLONG_MAX);
	}
	// get and open the trace file name for this router;
	init_local_injection_file();
	local_injection_file() >> _local_injection_time;

	// set the routing algo for this router as required thru the
	// topology file from user;
	_routing_algo = _vnoc->topology()->routing_algo();
}

void ROUTER::init_local_injection_file()
{
	pair <int, int> temp_pair;
	char temp_str[5];

	string name_t = _vnoc->topology()->trace_file();
	name_t.append(".");
	_assigned_core = _router_assignment->assigned(_address[0],_address[1]);
	sprintf(temp_str, "%d", _assigned_core);
	name_t.append(temp_str);
	name_t.append(".trs");

	_local_injection_file = new ifstream;
	_local_injection_file->open( name_t.c_str());

	if ( _local_injection_file->fail() ) {
		printf("\nError: Cannot open trace file: %s\n", name_t.c_str());
		exit(1);
	}
}

void ROUTER::receive_packet_from_trace_file()
{
	// retrieve packet from source trace file associated with this router;
	double event_time = _vnoc->event_queue()->current_time();
	long cube_size = _vnoc->topology()->cube_size();
	ADDRESS src_addr;
	ADDRESS des_addr;
	long packet_size;
	pair <int,int> temp_pair;
	int core_id;

	while ( ( _input.injection_buff_full() == false) && 
			( _local_injection_time <= event_time + S_ELPS)) {

		src_addr.clear();
		des_addr.clear();

		// translates a core id to a mesh XY address;
		// read source;
		local_injection_file() >> core_id;
		//assert( !local_injection_file().eof());
		if ( local_injection_file().eof()) {
			return;
		}

		temp_pair = _router_assignment->core_id_to_router_xy(core_id);
		src_addr.push_back(temp_pair.first);
		src_addr.push_back(temp_pair.second);
		assert( src_addr[0] < _ary_size && src_addr[1] < _ary_size);
	
		// read destination;
		local_injection_file() >> core_id;
		assert( !local_injection_file().eof());
		temp_pair = _router_assignment->core_id_to_router_xy(core_id);
		des_addr.push_back(temp_pair.first);
		des_addr.push_back(temp_pair.second);
		assert( (des_addr[0] < _ary_size) && (des_addr[1] < _ary_size));

		// read packet size;
		local_injection_file() >> packet_size;
		assert( !local_injection_file().eof());

		// LANDMARK HERE 2

		// inject this packet: src_addr -> des_addr;
		// injection load control; in 10 by 10 cycles some packets are NOT
		// going to be injected, but the events creation and files reading
		// will be left untouched;
		_vnoc->sfra()->set_skip_counter( (_vnoc->sfra()->skip_counter() + 1) % 10 );
		if ( _vnoc->sfra()->skip_counter() < _vnoc->sfra()->max_skip_counter()) {
			inject_packet( _packet_counter,	src_addr, des_addr,
				_local_injection_time, packet_size);
			_packet_counter ++;
			_vnoc->packets_count_incr();
		} else {
			_vnoc->_event_queue->undo_injection();
		}

		// second, create next EVG_ event
		if ( !local_injection_file().eof()) {
			local_injection_file() >> _local_injection_time;

			if ( local_injection_file().eof()) { return; }
		}
	}
}

void ROUTER::inject_packet( long flit_id, ADDRESS &sor_addr, ADDRESS &des_addr,
	double time, long packet_size)
{
	VC_TYPE vc_t;

	// Note: here, in order to account for the extra-link delay between
	// core and this router, we mimic it by subtracting the extra delay
	// from the delay that is recorded with the injected flit as the
	// time of injection; this initial "time" will be used at the time
	// when the flit will be consumed at its destination; this "time"
	// is basically the time when the flit left the actual core, before 
	// traversing the extra-link to the router;
	time -= _vnoc->Linear_delay( WIRE_DELAY, BASE_WIRE,
		extra_length() * _vnoc->sfra()->extra_links_timing_factor());
	if ( _vnoc->sfra()->sim_mode() == SIMULATED_ARCH_25D ||
		_vnoc->sfra()->sim_mode() == SIMULATED_ARCH_3D) {
		time -= TSV_VIA_DELAY;
	}

	for ( long l = 0; l < packet_size; l++) {

		DATA flit_data; // vector<unsigned long long>;
		for ( long i = 0; i < _flit_size; i++) {

			_init_data[i] = static_cast<DATA_ATOMIC_UNIT>( // "make up stuff";
				_init_data[i] * CORR_EFF + _vnoc->topology()->rng().flat_ull(0, ULLONG_MAX));
			flit_data.push_back( _init_data[i]);
		}

		// LANDMARK HERE 3

		if ( l == 0) {
			vc_t = pair<long, long>(0, _input.input_buff(0,0).size());
			// if it's the HEADER flit choose the shortest waiting vc queue;
			for ( long i = 0; i < _vc_number; i++) {
				long t = _input.input_buff(0,i).size();
				if ( t < vc_t.second) {
					vc_t = pair<long, long>(i, t);
				}
			}
			// if the input buffer is empty, set it to be ROUTING;
			if ( _input.input_buff(0, vc_t.first).size() == 0) {
				_input.vc_state_update(0, vc_t.first, ROUTING);
			}

			// if the input buffer has more than predefined flits, then
			// add the flits and flag it;
			if ( _input.input_buff(0, vc_t.first).size() > BUFF_BOUND) {
				// this is causing problems - look for "bug1";
				_input.set_injection_buff_full();
			}
			_input.add_flit( 0, (vc_t.first),
				FLIT(flit_id, FLIT::HEADER, sor_addr, des_addr, time, flit_data));
		}
		else if ( l < packet_size - 1) {
			_input.add_flit( 0, (vc_t.first),
				FLIT(flit_id, FLIT::BODY, sor_addr, des_addr, time, flit_data));
		}
		else {
			_input.add_flit( 0, (vc_t.first),
				FLIT(flit_id, FLIT::TAIL, sor_addr, des_addr, time, flit_data));
		}
		// power module writing here;
	}
}

void ROUTER::receive_flit_from_upstream(long port_id, long vc_id, FLIT &flit)
{
	// receive flit from upstream (neighboring) router;
	_input.add_flit( port_id, vc_id, flit);

	// power module writing here;
	if ( flit.type() == FLIT::HEADER) {
		if ( _input.input_buff(port_id, vc_id).size() == 1) {
			_input.vc_state_update( port_id, vc_id, ROUTING);
		}
	} else {
		if ( _input.vc_state(port_id, vc_id) == INIT) {
			_input.vc_state_update( port_id, vc_id, SW_AB);
		}
	}
}

void ROUTER::consume_flit(double time, const FLIT &flit)
{
	// receive (i.e., consume) one flit at the destination router;
	if ( flit.type() == FLIT::TAIL) {
		_vnoc->event_queue()->inc_finished_count();
		double delta_t = time - flit.start_time();

		// Note: here, in order to account for the extra-link delay between
		// this router and its hooked core, we add to delta_t the delay of 
		// the extra-link;
		delta_t += _vnoc->Linear_delay( WIRE_DELAY, BASE_WIRE,
			extra_length() * _vnoc->sfra()->extra_links_timing_factor());

		if ( _vnoc->sfra()->sim_mode() == SIMULATED_ARCH_25D ||
			_vnoc->sfra()->sim_mode() == SIMULATED_ARCH_3D) {
			delta_t += TSV_VIA_DELAY;
		}

		update_delay( delta_t);
	}
}

void ROUTER::receive_credit(long i, long j)
{
	_output.counter_next_r_inc(i, j);
}

void ROUTER::send_flit_to_out_buffer()
{
	// flit out buffer to the output buffer;
	for ( long i = 0; i < _physical_ports_count; i++) {
		for ( long j = 0; j < _vc_number; j++) {
			if ( _input.vc_state(i, j) == SW_TR) {
				VC_TYPE out_t = _input.selected_routing(i, j);
				_output.counter_next_r_dec(out_t.first, out_t.second);

				double event_time = _vnoc->event_queue()->current_time();
				if (i != 0) {
					ADDRESS cre_add_t = _address;
					long cre_pc_t = i;
					if ( (i % 2) == 0) {
						cre_pc_t = i - 1;
						cre_add_t[(i-1)/2] ++;
						if (cre_add_t[(i-1)/2] == _ary_size) {
							cre_add_t[(i-1)/2] = 0;
						}
					} else {
						cre_pc_t = i + 1;
						cre_add_t[(i-1)/2] --;
						if (cre_add_t[(i-1)/2] == -1) {
							cre_add_t[(i-1)/2] = _ary_size - 1;
						}
					}

					// physical link delay for the credit token to travel back;
					//double delay = CREDIT_DELAY;
					//double delay = _vnoc->h_credit_delay();
					double delay = _vnoc->Linear_delay( CREDIT_DELAY, BASE_WIRE, _link_length);
					_vnoc->event_queue()->add_event( EVENT(EVENT::CREDIT,
						event_time + delay, _address, cre_add_t, cre_pc_t, j));
				}

				long in_size_t = _input.input_buff(i,j).size();
				assert(in_size_t >= 1);
				FLIT flit_t( _input.get_flit(i,j));
				_input.remove_flit(i, j);
				// power stuff here;
				_output.add_flit(out_t.first, flit_t);
				if ( i == 0) {
					if ( _input.injection_buff_full() == true) {
						if ( _input.input_buff(0,j).size() < BUFF_BOUND) {
							_input.clear_injection_buff_full();
							// "bug1" happens here?
							receive_packet_from_trace_file();
						}
					}
				}
				_output.add_addr(out_t.first, out_t);
				if ( flit_t.type() == FLIT::TAIL) {
					_output.release(out_t.first, out_t.second);
				}
				if ( in_size_t > 1) {
					if ( flit_t.type() == FLIT::TAIL) {
						if ( _vnoc->topology()->vc_sharing_mode() == NOT_SHARED) {
							if (i != 0){
								if (in_size_t != 1) {
									cout<<i<<":"<<in_size_t<<endl;
								}
								assert(in_size_t == 1);
							}
						}
						_input.vc_state_update(i, j, ROUTING);
					} else {
						_input.vc_state_update(i, j, SW_AB);
					}
				} else {
					_input.vc_state_update(i, j, INIT);
				}
			}
		}
	}

}

void ROUTER::send_flit_via_physical_link(long i)
{
	// flit traversal through the link; is it pipelined?
	double event_time = _vnoc->event_queue()->current_time();
	if ( _output.out_buffer(i).size() > 0) {
		ADDRESS wire_add_t = _address;
		long wire_pc_t ;
		if ((i % 2) == 0) {
			wire_pc_t = i - 1;
			wire_add_t[(i - 1) / 2] ++;
			if (wire_add_t[(i-1) / 2] == _ary_size) {
				wire_add_t[(i-1) / 2] = 0;
			}
		} else {
			wire_pc_t = i + 1;
			wire_add_t[(i - 1) / 2] --;
			if (wire_add_t[(i-1) / 2] == -1) {
				wire_add_t[(i-1) / 2] = _ary_size - 1;
			}
		}
		FLIT flit_t( _output.get_flit(i));
		VC_TYPE outadd_t = _output.get_addr(i);
		// power stuff here;

		_output.remove_flit(i);
		_output.remove_addr(i);
		
		// physical link delay for the flit to travel forward;
		//double delay = WIRE_DELAY;
		//double delay = _vnoc->h_wire_delay();
		double delay = _vnoc->Linear_delay( WIRE_DELAY, BASE_WIRE, _link_length);
		_vnoc->event_queue()->add_event( EVENT(EVENT::LINK, 
			event_time + delay, _address, wire_add_t, wire_pc_t, 
			outadd_t.second, flit_t));
	}

}

void ROUTER::send_flits_via_physical_link()
{
	// send flits from output ports through links to 
	// downstream routers; flits traverse physical links here;
	for (long i = 1; i < _physical_ports_count; i++) {
		send_flit_via_physical_link(i);
	}
}

void ROUTER::simulate_one_router()
{
	// LANDMARK HERE 5

	// simulate all routing pipeline stages;
	// stage 5: flit traversal;
	send_flits_via_physical_link();
	// stage 4: flit output buffer;
	send_flit_to_out_buffer();
	// stage 3: switch arbitration;
	sw_arbitration_stage();
	// stage 2: vc arbitration
	vc_arbitration_stage();
	// stage 1: routing decision;
	routing_decision_stage();
}

void ROUTER::sanity_check() const
{
	// sanity checks;
	for ( long i = 0; i < _physical_ports_count; i ++) {
		for ( long j = 0; j < _vc_number; j ++) {
			if ( _input.input_buff(i,j).size() > 0) {
				cout << "Input is not empty" << endl;
			}
			if ( _input.vc_state(i,j) != INIT) {
				cout << "Input state is wrong" << endl;
			} cout << _output.counter_next_r(i,j) << ":";
			if ( _output.counter_next_r(i,j) != _buffer_size) {
				cout << "Output vc counter is wrong" << endl;
			}
			if ( _output.vc_usage(i,j) != ROUTER_OUTPUT::FREE) {
				cout << "Output is not free" << endl;
			}
			if ( _output.assigned_to(i,j) != VC_NULL) {
				cout << "Output is not reset" << endl;
			}
		}
		if ( _output.out_buffer(i).size() > 0) {
			cout << "Output temp buffer is not empty" << endl;
		}
		if ( _output.out_addr(i).size() > 0) {
			cout << "Output temp buffer is not empty" << endl;
		}
		if( _output.local_counter(i) != _out_buffer_size) {
			cout << "Output local counter is not reset" << endl;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// sw arbitration
//
////////////////////////////////////////////////////////////////////////////////

void ROUTER::sw_arbitration_stage() 
{
	// switch arbitration pipeline stage;
	map<long, vector<VC_TYPE> > vc_o_map;

	for ( long i = 0; i < _physical_ports_count; i++) {
		vector<long> vc_i_t;
		for ( long j = 0; j < _vc_number; j++) {
			if ( _input.vc_state(i, j) == SW_AB) {
				VC_TYPE out_t = _input.selected_routing(i, j);
				if (( _output.counter_next_r(out_t.first, out_t.second) > 0) &&
					( _output.local_counter(out_t.first) > 0)) {
					vc_i_t.push_back(j);
				}
			}
		}
		long vc_size_t = vc_i_t.size();
		if ( vc_size_t > 1) {
			long win_t = _vnoc->topology()->rng().flat_l(0, vc_size_t);
			VC_TYPE r_t = _input.selected_routing(i, vc_i_t[win_t]);
			vc_o_map[r_t.first].push_back(VC_TYPE(i, vc_i_t[win_t]));
		} else if ( vc_size_t == 1) {
			VC_TYPE r_t = _input.selected_routing(i, vc_i_t[0]);
			vc_o_map[r_t.first].push_back(VC_TYPE(i, vc_i_t[0]));
		}
	}

	if ( vc_o_map.size() == 0) {
		return;
	}

	for ( long i = 0; i < _physical_ports_count; i++) {
		long vc_size_t = vc_o_map[i].size();
		if ( vc_size_t > 0) {
			VC_TYPE vc_win = vc_o_map[i][0];
			if ( vc_size_t > 1) {
				vc_win = vc_o_map[i][ _vnoc->topology()->rng().flat_l(0, vc_size_t) ];
			}
			_input.vc_state_update(vc_win.first, vc_win.second, SW_TR);
			FLIT &flit_t = _input.get_flit(vc_win.first, vc_win.second);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// vc arbitration algorithms: implementation of selection function in routing
//
////////////////////////////////////////////////////////////////////////////////

pair<long, long> ROUTER::vc_selection(long i, long j) 
{
	// choose one of the candidate routing vc;
	vector<VC_TYPE > &vc_can_t = _input.routing(i, j);
	long r_size_t = vc_can_t.size();
	assert( r_size_t > 0);
	vector<VC_TYPE > vc_acq_t;

	for ( long i = 0; i < r_size_t; i++) {
		VC_TYPE v_t = vc_can_t[i];
		if ( _vnoc->topology()->vc_sharing_mode() == SHARED) {
			if ( _output.vc_usage(v_t.first, v_t.second) == ROUTER_OUTPUT::FREE) {
				vc_acq_t.push_back(vc_can_t[i]);
			}
		} else {
			if ( _output.counter_next_r(v_t.first, v_t.second) == _buffer_size) {
				if (  _output.vc_usage(v_t.first, v_t.second) == ROUTER_OUTPUT::FREE) {
					vc_acq_t.push_back(vc_can_t[i]);
				}
			}
		}
	}

	r_size_t = vc_acq_t.size();
	long vc_t = 0;
	if ( r_size_t > 0) {
		if ( r_size_t > 1) {
			vc_t = _vnoc->topology()->rng().flat_l(0, r_size_t);
		}
		return (vc_acq_t[vc_t]);
	} else {
		return VC_TYPE(-1, -1);
	}
}

void ROUTER::vc_arbitration_stage()
{
	map<VC_TYPE, vector<VC_TYPE> > vc_o_i_map;
	DATA_ATOMIC_UNIT vc_request = 0;

	for ( long i = 0; i < _physical_ports_count; i++) {
		for ( long j = 0; j < _vc_number; j++) {
			VC_TYPE vc_t;
			if ( _input.vc_state(i, j) == VC_AB) {
				vc_t = vc_selection(i,j);
				if ((vc_t.first >= 0) && (vc_t.second >= 0)) {
					vc_o_i_map[vc_t].push_back(VC_TYPE(i, j));
					vc_request = vc_request | VC_MASK[i * _vc_number + j];
				}
			}
		}
	}
	if ( vc_o_i_map.size() == 0) {
		return;
	}

	for ( long i= 1; i < _physical_ports_count; i++) {
		for ( long j = 0; j < _vc_number; j++) {
			if ( _output.vc_usage(i, j) == ROUTER_OUTPUT::FREE) {
				long cont_temp = vc_o_i_map[VC_TYPE(i,j)].size();
				if ( cont_temp > 0) {
					VC_TYPE vc_win = vc_o_i_map[VC_TYPE(i,j)][0];
					if ( cont_temp > 1) {
						vc_win = vc_o_i_map[VC_TYPE(i,j)][
							_vnoc->topology()->rng().flat_l(0, cont_temp)];
					}
					_input.vc_state_update(vc_win.first, vc_win.second, SW_AB);
					_input.assign_selected_routing(vc_win.first, 
						vc_win.second, VC_TYPE(i,j));
					_output.acquire(i, j, vc_win);
					// power stuff here;
				}
			}
		}
	}	
}

////////////////////////////////////////////////////////////////////////////////
//
// routing_decision_stage
//
////////////////////////////////////////////////////////////////////////////////

void ROUTER::routing_decision_stage()
{
	// LANDMARK HERE 6

	// only two-dimension is supported for now;
	double event_time = _vnoc->event_queue()->current_time();

	// injection physical port 0;
	for ( long j = 0; j < _vc_number; j++) {
		// HEADER flit;
		FLIT flit_t;
		if( _input.vc_state(0,j) == ROUTING) {
			flit_t = _input.get_flit(0,j);
			ADDRESS des_t = flit_t.des_addr();
			ADDRESS sor_t = flit_t.src_addr();
			if ( _address == des_t) {
				consume_flit( event_time, flit_t);
				_input.remove_flit(0, j);
				_input.vc_state_update(0, j, HOME);
			} else {
				_input.clear_routing(0,j);
				_input.clear_selected_routing(0,j);

				// call the actual routing algo;
				call_current_routing_algorithm( des_t, sor_t, 0, j);

				_input.vc_state_update(0, j, VC_AB);
			}
		// BODY or TAIL type flits;
		} else if ( _input.vc_state(0,j) == HOME) {
			if ( _input.input_buff(0, j).size() > 0) {
				flit_t = _input.get_flit(0, j);
				assert( flit_t.type() != FLIT::HEADER);
				consume_flit( event_time, flit_t);
				_input.remove_flit(0, j);
				if ( flit_t.type() == FLIT::TAIL) {
					if ( _input.input_buff(0, j).size() > 0) {
						_input.vc_state_update(0, j, ROUTING);
					} else {
						_input.vc_state_update(0, j, INIT);
					}
				}
			}
		}
	}

	// other physical ports;
	for ( long i = 1; i < _physical_ports_count; i++) {
		for ( long j = 0; j < _vc_number; j++) {
			// send back CREDIT event/message;
			FLIT flit_t;
			if ( _input.input_buff(i,j).size() > 0) {
				flit_t = _input.get_flit(i,j);
				ADDRESS des_t = flit_t.des_addr();
				if ( _address == des_t) {
					ADDRESS cre_add_t = _address;
					long cre_pc_t = i;
					// if the destination is to the right, send credit to
					// left and vice-versa; if it's up, send it down and vice-versa;
					if ((i % 2) == 0) {
						cre_pc_t = i - 1;
						cre_add_t[(i-1)/2] ++;
						if (cre_add_t[(i-1)/2] == _ary_size) {
							cre_add_t[(i-1)/2] = 0;
						}
					} else {
						cre_pc_t = i + 1;
						cre_add_t[(i-1)/2] --;
						if (cre_add_t[(i-1)/2] == -1) {
							cre_add_t[(i-1)/2] = _ary_size - 1;
						}
					}

					// physical link delay;
					//double delay = CREDIT_DELAY;
					//double delay = _vnoc->h_credit_delay();
					double delay = _vnoc->Linear_delay( CREDIT_DELAY, BASE_WIRE, _link_length);
					_vnoc->event_queue()->add_event( EVENT(EVENT::CREDIT,
						event_time + delay, _address, cre_add_t, cre_pc_t, j));
				}
			}
			//	HEADER flit;
			if ( _input.vc_state(i, j) == ROUTING) {
				flit_t = _input.get_flit(i, j);
				assert(flit_t.type() == FLIT::HEADER);

				// ROUTING state only when the header passes through;
				ADDRESS des_t = flit_t.des_addr();
				ADDRESS sor_t = flit_t.src_addr();
				if ( _address == des_t) {
					consume_flit( event_time, flit_t);
					_input.remove_flit(i, j);
					_input.vc_state_update(i, j, HOME);
				} else {
					_input.clear_routing(i, j);
					_input.clear_selected_routing(i, j);

					// call the actual routing algo;
					call_current_routing_algorithm( des_t, sor_t, i, j);

					_input.vc_state_update(i, j, VC_AB);
				}
			// BODY or TAIL flits;
			} else if ( _input.vc_state(i, j) == HOME) {
				if ( _input.input_buff(i, j).size() > 0) {
					flit_t = _input.get_flit(i, j);
					assert( flit_t.type() != FLIT::HEADER);
					consume_flit( event_time, flit_t);
					_input.remove_flit(i, j);
					if ( flit_t.type() == FLIT::TAIL) {
						if ( _input.input_buff(i, j).size() > 0) {
							_input.vc_state_update(i, j, ROUTING);
						} else {
							_input.vc_state_update(i, j, INIT);
						}
					}
				}
			}
		}
	}
}

void ROUTER::call_current_routing_algorithm(
	const ADDRESS &des_t, const ADDRESS &sor_t, 
	long s_ph, long s_vc)
{
	// setup _routing matrix that implements currently used 
	// routing algo (XY or TXY);

	long xoffset = des_t[0] - _address[0];
	long yoffset = des_t[1] - _address[1];

	// (1) XY
	if ( _routing_algo == ROUTING_XY) {

		if ( yoffset < 0) {
			_input.add_routing(s_ph, s_vc, VC_TYPE(3,0));
			_input.add_routing(s_ph, s_vc, VC_TYPE(3,1));
			_input.add_routing(s_ph, s_vc, VC_TYPE(3,2));
			_input.add_routing(s_ph, s_vc, VC_TYPE(3,3));
		} else if ( yoffset > 0) {
			_input.add_routing(s_ph, s_vc, VC_TYPE(4,0));
			_input.add_routing(s_ph, s_vc, VC_TYPE(4,1));
			_input.add_routing(s_ph, s_vc, VC_TYPE(4,2));
			_input.add_routing(s_ph, s_vc, VC_TYPE(4,3));
		} else {
			if ( xoffset < 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(1,0));
				_input.add_routing(s_ph, s_vc, VC_TYPE(1,1));
				_input.add_routing(s_ph, s_vc, VC_TYPE(1,2));
				_input.add_routing(s_ph, s_vc, VC_TYPE(1,3));
			} else if ( xoffset > 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(2,0));
				_input.add_routing(s_ph, s_vc, VC_TYPE(2,1));
				_input.add_routing(s_ph, s_vc, VC_TYPE(2,2));
				_input.add_routing(s_ph, s_vc, VC_TYPE(2,3));
			}
		}
	}
	// (2) TXY
	else if ( _routing_algo == ROUTING_TXY) {

		bool xdirection = (abs(static_cast<int>(xoffset)) * 2 <= _ary_size) ? true: false; 
		bool ydirection = (abs(static_cast<int>(yoffset)) * 2 <= _ary_size) ? true: false; 

		if ( xdirection) {
			if ( xoffset < 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(1, 0));
			} else if ( xoffset > 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(2, 1));
			} else {
				if ( ydirection) {
					if ( yoffset < 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(3, 0));
					} else if ( yoffset > 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(4, 1));
					}
				} else {
					if ( yoffset < 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(4, 0));
					} else if ( yoffset > 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(3, 1)); 
					}
				}
			}
		} else {
			if ( xoffset < 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(2, 0));
			} else if ( xoffset > 0) {
				_input.add_routing(s_ph, s_vc, VC_TYPE(1, 1));
			} else {
				if ( ydirection) {
					if ( yoffset < 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(3, 0));
					} else if ( yoffset > 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(4, 1));
					}
				} else {
					if ( yoffset < 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(4, 0));
					} else if ( yoffset > 0) {
						_input.add_routing(s_ph, s_vc, VC_TYPE(3, 1)); 
					}
				}
			}
		}
	}
	
}

////////////////////////////////////////////////////////////////////////////////
//
// VNOC
//
////////////////////////////////////////////////////////////////////////////////

VNOC::VNOC( TOPOLOGY *topology, EVENT_QUEUE *event_queue,
	ROUTER_ASSIGNMENT *router_assignment, bool verbose) :
	_routers(),
	_ary_size(0),
	_cube_size(0),
	_routers_count(0),
	_packets_count(0),
	_input_file_st(),
	_verbose(verbose)
{
	_router_assignment = router_assignment;
	_topology = topology;
	_event_queue = event_queue;
	_ary_size = _topology->network_size();
	_cube_size = _topology->cube_size();

	_h_wire_delay = 1.0;
	_h_pipe_delay = 1.0;
	_h_credit_delay = 1.0;

	long vc_number = _topology->virtual_channel_number();
	long buffer_size = _topology->input_buffer_size();
	long output_buffer_size = _topology->output_buffer_size();
	long flit_size = _topology->flit_size();

	// + 1 means, one for injection;
	long phy_ports_t = _cube_size * 2 + 1;
	_routers_count = _ary_size;
	for ( long i = 0; i < _cube_size - 1; i++) {
		_routers_count = _routers_count * _ary_size;
	}
	ADDRESS add_t;
	add_t.resize( _cube_size, 0);

	for ( long i = 0; i < _routers_count; i++) {

		_routers.push_back(
			ROUTER( phy_ports_t, vc_number, buffer_size,
				output_buffer_size, add_t, _ary_size, flit_size,
				this, i,
				_router_assignment, _topology->link_length()));
		
		// assign the address of this router;
		add_t[_cube_size - 1]++;
		for ( long j = _cube_size -1; j > 0; j--) {
			if ( add_t[j] == _ary_size) {
				add_t[j] = 0;
				add_t[j-1]++;
			}
		}
	}
	init_input_file();
}

void VNOC::init_input_file() 
{
	string filename;
	filename=_topology->trace_file().c_str();
	filename.append(".trs");
	_input_file_st.open( filename.c_str());
	if ( !_input_file_st) {
		printf("\nError: Cannot open source file: %s\n",filename.c_str());
		exit(1);
	}
	double event_time_t;
	_input_file_st >> event_time_t;

	_event_queue->add_event( EVENT(EVENT::PE, event_time_t));
}

ROUTER & VNOC::router( const ADDRESS &a)
{
	ADDRESS::const_iterator first = a.begin();
	ADDRESS::const_iterator last = a.end();
	long i = (* first); first++;
	for (; first!= last; first++) {
		i = i * _ary_size + (*first);
	}
	return ( _routers[i]);
}

const ROUTER & VNOC::router( const ADDRESS &a) const
{
	ADDRESS::const_iterator first = a.begin();
	ADDRESS::const_iterator last = a.end();
	long i = (* first); first++;
	for (; first!= last; first++) {
		i = i * _ary_size + (*first);
	}
	return ( _routers[i]);
}

bool VNOC::check_address(const ADDRESS &a) const 
{
	// evaluate the address;
	if ( a.size() != _cube_size) {
		return false;
	}
	for ( long i = 0; i < a.size(); i++) {
		if ((a[i] >= _ary_size) || (a[i] < 0)) {
			return false;
		}
	}
	return true;
}

bool VNOC::receive_EVENT_PE()
{
	// first, inject the flits; 
	ADDRESS src_addr_t;
	ADDRESS des_addr_t;
	ADDRESS next_src_addr_t;
	long pack_size_t;
	pair <int,int> temp_pair;
	int core_id;

	// translates a core id to a mesh XY address;
	// read source;
	_input_file_st >> core_id;
	assert( !_input_file_st.eof());
	temp_pair = _router_assignment->core_id_to_router_xy(core_id);
	src_addr_t.push_back(temp_pair.first);
	src_addr_t.push_back(temp_pair.second);
	assert( src_addr_t[0] < _ary_size && src_addr_t[1] < _ary_size);

	// read destination;
	_input_file_st >> core_id;
	assert( !_input_file_st.eof());
	temp_pair = _router_assignment->core_id_to_router_xy(core_id);
	des_addr_t.push_back(temp_pair.first);
	des_addr_t.push_back(temp_pair.second);
	assert( (des_addr_t[0] < _ary_size) && (des_addr_t[1] < _ary_size));

	_input_file_st >> pack_size_t;
	assert( !_input_file_st.eof());

	// LANDMARK HERE 1

	router(src_addr_t).receive_packet_from_trace_file();

    if ( !_input_file_st.eof()) {
        double event_time_t;
		_input_file_st >> event_time_t;
		if ( !_input_file_st.eof()) {

			_event_queue->add_event( EVENT(EVENT::PE, event_time_t));
		}
	}

}

bool VNOC::receive_EVENT_ROUTER( EVENT this_event)
{
	// router pipeline stage;
	// delay through the router pipeline; it also has to be adjusted
	// to be proportional with the adjustable clock period given by
	// different link lengths; Tclk impacts: link delay, credit delay,
	// and pipe delay;
	//double delay = PIPE_DELAY;
	//double delay = _h_pipe_delay;
	double delay = Linear_delay( PIPE_DELAY, BASE_WIRE, _topology->link_length());
	_event_queue->add_event( EVENT(EVENT::ROUTER,this_event.start_time() + delay));

	for ( long i = 0; i < _routers_count; i++) {
		_routers[i].simulate_one_router(); // including power;
	}
}

bool VNOC::receive_EVENT_LINK( EVENT this_event)
{
	ADDRESS des_t = this_event.des_addr();
	long pc_t = this_event.pc();
	long vc_t = this_event.vc();
	FLIT &flit = this_event.flit();
	router(des_t).receive_flit_from_upstream(pc_t, vc_t, flit);
}

bool VNOC::receive_EVENT_CREDIT( EVENT this_event)
{
	ADDRESS des_t = this_event.des_addr();
	long pc_t = this_event.pc();
	long vc_t = this_event.vc();
	router(des_t).receive_credit(pc_t, vc_t);
}

void VNOC::check_simulation() 
{
	// check if the network is back to the inital state?
	vector<ROUTER>::const_iterator first = _routers.begin();
	vector<ROUTER>::const_iterator last = _routers.end();
	for (; first != last; first++) {
		first->sanity_check(); // empty check;
	}
	printf("\nSuccess: check for routers cleaness is ok\n");
}

void VNOC::print_network_routers() 
{
}

void VNOC::print_simulation_results() 
{
	vector<ROUTER>::const_iterator first = _routers.begin();
	vector<ROUTER>::const_iterator last = _routers.end();
	double total_delay = 0;
	// calculate the total delay
	first = _routers.begin();
	for (; first != last; first++) {
		total_delay += first->total_delay();
	}
	long tot_f_t = _event_queue->finished_count();

	first = _routers.begin();
	last = _routers.end();
	for (; first != last; first++) {
		// accumulate power stuff here;
	}
	double curr_time = _event_queue->current_time();
	// compute averages here;

	double delay_scale_factor = 1; // _h_wire_delay / WIRE_DELAY;

	// store here result; will be retrieved by any application
	// of the vNOC simulator; 
	_latency = total_delay / max(tot_f_t, long(1));
	_latency *= delay_scale_factor;
	// in fact store it directly in its sketch counterpart of
	// the sfra host object;
	_sfra->set_sketch_latency( _latency);
	// record also the avg. number of packets per cycle; used for plots;
	_sfra->set_sketch_packets_per_cycle( double(_packets_count)/curr_time);

	cout.precision(6);
	cout << "--------------------------------------------------" << endl;
	cout << "total number of flits delivered:		" << tot_f_t << endl;
	cout << "average delay per flit:				" << _latency << endl;
	cout << "--------------------------------------------------" << endl;
}

bool VNOC::run_simulation()
{
	bool result;
	
	result = _event_queue->run_simulation();

	return result;
}

////////////////////////////////////////////////////////////////////////////////
//
// VNOC MISC
//
////////////////////////////////////////////////////////////////////////////////

double VNOC::Elmore_delay( double base_delay, double base_length, double actual_length)
{
	// delay is proportional to the square of the lengths ratio;
	// calculation is done based on empirical (transmission line simulation) data
	// for a given base wire length we know that we have a certain base delay;
	return base_delay * (actual_length/base_length)*(actual_length/base_length);
}

double VNOC::Linear_delay( double base_delay, double base_length, double actual_length)
{
	// delay is proportional to the length; this is true when the interconnects are
	// buffered; we assume that that is the case as in todays technology nodes;
	// otherwise, results - as flit latency - will have a large std. variation;
	return base_delay * (actual_length/base_length);
}

double VNOC::Via_delay( double base_delay, double layers) 
{
	// via delay is based on Elmore delay (base reference is one layer 
	// traversal); currently NOT used;
	//return Elmore_delay(VIA_DELAY, 1, layers);
	return 0.0;
}

void VNOC::compute_heterogeneous_t_clock( double max_link_length,
	bool square_proportionality)
{
	// compute the extrapolated values for: WIRE_DELAY, PIPE_DELAY, and
	// CREDIT_DELAY;
	if ( square_proportionality) { // delay is SQUARE with link length;
		_h_wire_delay = Elmore_delay( WIRE_DELAY, BASE_WIRE, // 0.9, 1000 um
			max_link_length);
		_h_pipe_delay = Elmore_delay( PIPE_DELAY, BASE_WIRE, // 1.0, 1000 um
			max_link_length);
		_h_credit_delay = Elmore_delay( CREDIT_DELAY, BASE_WIRE, // 1.0, 1000 um
			max_link_length);
		//printf("\nT clock: %.2f", _h_wire_delay);
		//printf("\nLink max length: %.2f\n", max_link_length);
	} else { // delay is LINEAR with link length;
		_h_wire_delay = Linear_delay( WIRE_DELAY, BASE_WIRE, // 0.9, 1000 um
			max_link_length);
		_h_pipe_delay = Linear_delay( PIPE_DELAY, BASE_WIRE, // 1.0, 1000 um
			max_link_length);
		_h_credit_delay = Linear_delay( CREDIT_DELAY, BASE_WIRE, // 1.0, 1000 um
			max_link_length);
	}
}
