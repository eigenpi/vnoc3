#include "config.h"
#include "sfra.h"
#include "vnoc_event.h"
#include <assert.h>

#ifdef BUILD_WITH_GUI
#include "sfra_gui.h"
#endif


using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// EVENT_QUEUE
//
////////////////////////////////////////////////////////////////////////////////

EVENT_QUEUE::EVENT_QUEUE(double start_time, TOPOLOGY *topology) :
	_last_time(0),
	_event_count(0), 
	_finished_count(0), 
	_events()
{
	_current_time = start_time;
	_topology = topology;

	add_event( EVENT(EVENT::ROUTER, 0.0));
}

void EVENT_QUEUE::undo_injection(){
	_events_injected_count--;
}

bool EVENT_QUEUE::run_simulation() 
{
	double report_at_time = 0;
	_events_injected_count = 0;
	char msg[BUFFER_SIZE];

	// gui to be or not to be;
	#ifdef BUILD_WITH_GUI
	if ( _topology->use_gui()) {
		sprintf( msg, "INITIAL - Time: %.2f  Remaining: %.2f  Injected: %ld  Finished: %ld ",
			0.0, _topology->simulation_cycles_count(), 0, 0);
		// changed from PRIORITY_MAJOR to PRIORITY_MINOR to avoid the need 
		// of clicking on "Proceed" button;
		_vnoc->gui()->update_screen( PRIORITY_MINOR, msg, ROUTERS);
	}
	#endif

	// pick up events from queue and process as long as it's not empty or
	// forced to stop;
	double simulation_cycles_count = _topology->simulation_cycles_count();
	while ( _events.size() > 0 && _current_time <= simulation_cycles_count) {

		EVENT this_event = *get_event();
		remove_top_event();
		assert( _current_time <= _current_time + S_ELPS);
		_current_time = this_event.start_time();

		if ( _current_time > report_at_time) {
			if ( _vnoc->verbose()) {
				printf("Current time: %.2f  Remaining time %.2f\nInjected packets: %ld  Finished packets count: %ld\n",
					_current_time, (_topology->simulation_cycles_count() - _current_time),
					_events_injected_count, _finished_count);
		
				_vnoc->print_simulation_results();
			}

			report_at_time += REPORT_STATS_PERIOD;

			// gui to be or not to be;
			#ifdef BUILD_WITH_GUI
			if ( _topology->use_gui()) {
				double left_time = _topology->simulation_cycles_count() - _current_time;
				sprintf( msg, "Current time: %.2f  Remaining: %.2f  Injected: %ld  Finished: %ld ",
						 _current_time, (left_time > 0 ? left_time : 0.0),
					_events_injected_count, _finished_count);
				_vnoc->gui()->update_screen( ( _topology->user_step_by_step() ? 
					PRIORITY_MAJOR : PRIORITY_MINOR), msg, ROUTERS);
			}
			#endif
		}
		
		switch ( this_event.type()) {

			case EVENT::PE :
				_vnoc->receive_EVENT_PE();
				_events_injected_count ++;
				break;

			case EVENT::ROUTER :
				_vnoc->receive_EVENT_ROUTER( this_event);
				break;

			case EVENT::LINK :
				_vnoc->receive_EVENT_LINK( this_event);
				break;

			case EVENT::CREDIT :
				_vnoc->receive_EVENT_CREDIT( this_event);
				break;

			default:
				printf("DUMMY\n");
				assert(0);
				break;
		} 
	}

	// gui to be or not to be;
	#ifdef BUILD_WITH_GUI
	if ( _topology->use_gui()) {
		//sprintf( msg, "FINAL - Time: %.2f  Remaining: %.2f  Injected: %ld  Finished: %ld ",
		//	_current_time, 0.0, _events_injected_count, _finished_count);
		sprintf( msg, "");
		_vnoc->gui()->update_screen( PRIORITY_MAJOR, msg, ROUTERS);
	}
	#endif
}
