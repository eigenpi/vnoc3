#ifndef sa_btreeH
#define sa_btreeH

#include <config.h>
#include "fp_plan.h"


////////////////////////////////////////////////////////////////////////////////
//
// SA_FLOORPLANING
//
////////////////////////////////////////////////////////////////////////////////

class SA_FLOORPLANING {
 private:
	// floorplaning annealing parameters; these were globals in the BTree code;
	// here this is just a host wrapper; their values will be imported from the
	// "sfra" host object;
	int _times;
	int _local;
	float _term_temp;
	float _avg_ratio;
	float _lambda;
	float _alpha;
	float _fp_scale;
	// next two are always with default values; not really changed currently by user;
	int _hill_climb_stage;
	float _init_avg;

 public:
	FPlan *_fp_p;
	// if _verbose is true then detailed debug info will be printed;
	bool _verbose;

 public:
	SA_FLOORPLANING() {
		// all defaults;
		_times = 160;
		_local = 6;
		_term_temp = 0.1; 
		_avg_ratio = 30; // had default value 150 in the original BTree?
		_lambda = 1.3;
		_alpha = 0.25;
		_fp_scale = 1.0;
		_hill_climb_stage = 7;
		_init_avg = 0.00001;
		_verbose = true;
	}
	SA_FLOORPLANING( FPlan *fp_p, int times, int local, float term_temp) {
		_fp_p = fp_p;
		_times = times;
		_local = local;
		_term_temp = term_temp;
		// rest of them defaults;
		_avg_ratio = 30; // had default value 150 in the original BTree?
		_lambda = 1.3;
		_alpha = 0.25;
		_fp_scale = 1.0;
		_hill_climb_stage = 7;
		_init_avg = 0.00001;
		_verbose = true;
	}
	~SA_FLOORPLANING() {}

	bool verbose() const { return _verbose; }
	void set_verbose(bool v) { _verbose = v; }
	void set_times(int val) { _times = val; }
	void set_local(int val) { _local = val; }
	void set_term_temp(float val) { _term_temp = val; }
	void set_avg_ratio(float val) { _avg_ratio = val; }
	void set_lambda(float val) { _lambda = val; }
	void set_alpha(float val) { _alpha = val; }
	void set_fp_scale(float val) { _fp_scale = val; }

	void run_SA_Floorplaning();
	// some utils functions;
	double mean( vector<double> &chain);
	double std_var(vector<double> &chain);
};

#endif
