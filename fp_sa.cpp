#include <config.h>
#include <iostream>
#include <assert.h>
#include <cmath>
#include "fp_sa.h"

using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// SA_FLOORPLANING
//
////////////////////////////////////////////////////////////////////////////////

// initial original version:
// double SA_Floorplan(FPlan &fp, int k, int local, float term_T) {...}
//
// Simulated Annealing B*Tree Floorplan
//	 k: factor of the number of permutation in one temperature
//	 local: local search iterations
//	 termT: terminating temperature

void SA_FLOORPLANING::run_SA_Floorplaning()
{
	// I should make it return the runtime it takes;

	int MT, uphill, reject;
	double pre_cost, best,cost;
	float d_cost,reject_rate;
  
	int N = _times * _fp_p->size(); // _times used to be argument "k";
	float P = 0.9;
	float T, actual_T = 1;
	double avg = _init_avg;
	float conv_rate = 1;

	double estimate_avg = 0.08 / _avg_ratio;
	cout << "Estimate Average Delta Cost = " << estimate_avg << endl;

	if ( _local == 0) {
		avg = estimate_avg;
	}
	
	T = avg / log(P);  
  
	// get inital solution
	_fp_p->packing();
	_fp_p->keep_sol();	
	_fp_p->keep_best();
	pre_cost = best = _fp_p->getCost();
  
	int good_num=0,bad_num=0;
	double total_cost=0;
	int count=0;
	ofstream of("/tmp/btree_debug");

	do {
		count++;
		total_cost = 0;
		MT=uphill=reject=0;
		if ( _verbose) {
			printf("Iteration %d, T= %.2f\n", count, actual_T);
		}
		vector<double> chain; 
 
		for (; uphill < N && MT < 2*N; MT++) {
			_fp_p->perturb();
			_fp_p->packing();
			cost = _fp_p->getCost(); 
			d_cost = cost - pre_cost;
			float p = exp(d_cost/T);
	   

			chain.push_back(cost);

			if ( d_cost <=0 || rand_01() < p ) {
				_fp_p->keep_sol();
				pre_cost = cost;

				if ( d_cost > 0){		  
					uphill++, bad_num++;
					of << d_cost << ": " << p << endl;
				} else if(d_cost < 0)  good_num++;

				// keep best solution
				if ( cost < best){
					_fp_p->keep_best();
					best = cost;
					if ( _verbose) {
						printf("   ==>	Cost= %f, Area= %.6f, ", best, _fp_p->getArea()*1e-6);
						printf("Wire= %.3f\n", _fp_p->getWireLength()*1e-3);
					}
					assert(_fp_p->getArea() >= _fp_p->getTotalArea());
				}
			}
			else {
				reject++;
				_fp_p->recover();
			}
		}
		//	 cout << T << endl;
		double sv = std_var(chain);
		float r_t = exp( _lambda * T / sv);
		T = r_t*T;


		// After apply local-search, start to use normal SA
		if ( count == _local){
			T = estimate_avg/ log(P);
			T *= pow(0.9, _local);		// smothing the annealing schedule
			actual_T = exp(estimate_avg/T);
		}
		if ( count > _local){
			actual_T = exp(estimate_avg/T);
			conv_rate = 0.95;
		}

		reject_rate = float(reject)/MT;
		if ( _verbose) {
			printf("  T= %.2f, r= %.2f, reject= %.2f\n\n", actual_T, r_t, reject_rate);
		}
		
	} while (reject_rate < conv_rate && actual_T > _term_temp);

	if (reject_rate >= conv_rate)
		cout << "\n	 Convergent!\n";
	else if (actual_T <= _term_temp)
		cout << "\n Cooling Enough!\n";

	printf("\n good = %d, bad=%d\n\n", good_num, bad_num);

	_fp_p->recover_best();
	_fp_p->packing();

}

// not used, and still using "FPlan &fp" instead of a pointer;
// double SA_FLOORPLANING::Random_Floorplan( FPlan &fp, int times) {...}

double SA_FLOORPLANING::mean( vector<double> &chain)
{
	double sum = 0;
	for ( int i = 0; i < chain.size(); i++)
		sum += chain[i];
	return sum / chain.size();
}

double SA_FLOORPLANING::std_var(vector<double> &chain)
{
	double m = mean(chain);
	double sum = 0;
	double var;
	int N = chain.size();
	for ( int i = 0; i < N; i++) {
		sum += (chain[i]-m)*(chain[i]-m);
	}
	var = sqrt(sum/(N-1));
	if ( _verbose) {
		printf("  m=%.4f ,v=%.4f\n",m,var);
	}
	return var;
}
