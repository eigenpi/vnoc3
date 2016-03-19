#ifndef _VNOC_EVENT_H_
#define _VNOC_EVENT_H_

#include "config.h"
#include "vnoc_topology.h"
#include "vnoc.h"
#include <set>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// EVENT
//
////////////////////////////////////////////////////////////////////////////////

class EVENT {
	public:
		enum EVENT_TYPE { PE, ROUTER, LINK, CREDIT, DUMMY };
	private:
		EVENT_TYPE _type;
		ADDRESS _src_addr;
		ADDRESS _des_addr;
		double _start_time;
		long _pc;
		long _vc;
		FLIT _flit;

	public:
		// PE and ROUTER message events;
		EVENT( EVENT_TYPE type, double time) : _type(type), _start_time(time),
			_pc(), _vc(), _src_addr(), _des_addr(), _flit() { }
		// CREDIT message event;
		EVENT( EVENT_TYPE type, double time,
			ADDRESS &s, ADDRESS &d, long pc, long vc) : _type(type),
			_start_time(time), _src_addr(s), _des_addr(d),
			_pc(pc), _vc(vc), _flit() { }
		// LINK message event;
		EVENT( EVENT_TYPE type, double time, ADDRESS &s, ADDRESS &d,
			long pc, long vc, FLIT &flit) : _type(type), _start_time(time),
			_src_addr(s), _des_addr(d), _pc(pc), _vc(vc), _flit(flit) { }
		EVENT( EVENT_TYPE type, double time, FLIT &flit) : _type(type),
			_start_time(time), _pc(), _vc(), 
			_src_addr(), _des_addr(), _flit(flit) { }
		EVENT( EVENT &event) : _type(event.type()),
			_start_time(event.start_time()),
			_src_addr(event.src_addr()), _des_addr(event.des_addr()),
			_pc(event.pc()), _vc(event.vc()), _flit(event.flit()) { }
		EVENT( const EVENT &event) : _type(event.type()),
			_start_time(event.start_time()),
			_src_addr(event.src_addr()), _des_addr(event.des_addr()),
			_pc(event.pc()), _vc(event.vc()), _flit(event.flit()) { }
		~EVENT() {}


		EVENT_TYPE type() const { return _type; }
		ADDRESS src_addr() const { return _src_addr; }
		ADDRESS des_addr() const { return _des_addr; }
		double start_time() const { return _start_time; }
		long pc() const { return _pc; }
		long vc() const { return _vc; }
		FLIT &flit() { return _flit; }
		const FLIT &flit() const { return _flit;}
};

////////////////////////////////////////////////////////////////////////////////
//
// EVENT_QUEUE
//
////////////////////////////////////////////////////////////////////////////////

inline bool operator<(const EVENT &a, const EVENT &b) {
	return a.start_time() < b.start_time();
}

class EVENT_QUEUE {
	private:
		multiset<EVENT> _events;
		double _current_time;
		double _last_time; 
		long _event_count;
		// total number of flits succesfully carried to their destination;
		long _finished_count;
		long _events_injected_count;
	public:
		TOPOLOGY *_topology;
		VNOC *_vnoc;

	public:
		EVENT_QUEUE( double start_time, TOPOLOGY *topology);
		~EVENT_QUEUE() {}

		typedef multiset<EVENT>::size_type size_type; 
		typedef multiset<EVENT>::iterator iterator;

		VNOC *vnoc() { return _vnoc; }
		void set_vnoc(VNOC *vnoc) { _vnoc = vnoc; };
		TOPOLOGY *topology() const { return _topology; }
		double current_time() const { return _current_time; }
		double last_time() const { return _last_time; }
		long event_count() const { return _event_count; }
		long finished_count() const { return _finished_count; }
		void inc_finished_count() { _finished_count ++; }
		iterator get_event() { return _events.begin(); }
		void remove_event( iterator pos) { _events.erase(pos); }
		void remove_top_event() { _events.erase(_events.begin()); }
		size_type event_queue_size() const { return _events.size(); }
		void add_event( const EVENT &event) { _event_count ++; 
			_events.insert(event); }
		void undo_injection();

		bool run_simulation();
};

#endif
