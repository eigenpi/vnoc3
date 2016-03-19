#include <config.h>
#include <iostream>
#include <assert.h>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <climits>
#include <sys/time.h>
#include <list>
#include <cmath>
#include <sstream>
#include "sfra.h"
#include "fp_plan.h"
#include "vnoc_topology.h"

using namespace std;

class SFRA;


// I should get rid of these globals;
char line[100],t1[40],t2[40];
ifstream fs;


////////////////////////////////////////////////////////////////////////////////
//
// FPlan
//
////////////////////////////////////////////////////////////////////////////////

FPlan::~FPlan() {
	fs.close();
}

FPlan::FPlan(float calpha=1)
{
	norm_area = 1;
	norm_wire = 1;
	cost_alpha = calpha;
	_sfra = 0; // has to be set by calling set_sfra_host();
	// reset the id of the root module to -1; this will mean that
	// there is no root module with pins and global nets to be used 
	// for WL calculations; this will be the case of subfloorplans only;
	root_module.id = -1;
}

void FPlan::inflate_core( int id, int routers)
{
	// expand the dimensions of a core as if a certain number of routers 
	// were being implemented inside it (in the same plane);
	double old_area, new_area, area_ratio, dimensions_ratio;
	old_area = modules[id].width * modules[id].height;
	new_area = old_area + ROUTER_AREA * routers;
	area_ratio = new_area / old_area;
	dimensions_ratio = sqrt(area_ratio);
	modules[id].width = (int) (modules[id].width * dimensions_ratio);
	modules[id].height = (int) (modules[id].height * dimensions_ratio);
	modules[id].area = int( new_area);
}

void FPlan::packing()
{
	if (cost_alpha!=1)
		calcWireLength();
}

void FPlan::clear()
{
	Area = 0; 
	WireLength = 0;
}

double FPlan::getCost()
{
	if(cost_alpha==1)
		return cost_alpha*(Area/norm_area);
	else if(cost_alpha < 1e-4)
		return (WireLength/norm_wire);
	else
		return cost_alpha*(Area/norm_area)+(1-cost_alpha)*(WireLength/norm_wire);
}

float FPlan::getDeadSpace()
{
	return 100*(Area-TotalArea)/float(Area);
}

void FPlan::normalize_cost(int t)
{
	norm_area=norm_wire=0;

	for (int i=0; i < t; i++) {
		perturb();
		packing(); // Note: B_Tree has packing() too;
		norm_area += Area;
		norm_wire += WireLength;
	}
  
	norm_area /= t;
	norm_wire /= t;
	printf("normalize area=%.0f, wire=%.0f\n", norm_area, norm_wire);
}

////////////////////////////////////////////////////////////////////////////////
//
// read from file
//
////////////////////////////////////////////////////////////////////////////////

char *tail(char *str)
{
	str[strlen(str)-1]=0;
	return str;
}

void FPlan::read( char *file, float scale)
{
	// testcase file has .fp extension; open it and read module by module;
	char fp_file[256];
	strcpy(fp_file,file);
	strcat(fp_file,".fp");

	filename = fp_file; 
	fs.open(fp_file);
	if (fs==NULL)
		error("Error:\tUnable to open file: %s",fp_file);
	fs.clear(); // needed for a big loop in Windows;

	bool final=false;
	Module dummy_mod;


	// (1) part 1: read IP/core modules and the top level module;
	for (int i=0; !fs.eof(); i++) {

		// (a) new module;
		// example:
		// MODULE cc_11;
		modules.push_back(dummy_mod);	// new module
		Module &mod = modules.back();
		mod.id = i;
		mod.pins.clear();

		fs >> t1 >> t2;
		tail(t2);			// remove ";"
		strcpy(mod.name,t2);

		fs >> t1 >> t2;
		if(!strcmp(t2,"PARENT;"))
			final = true;
	
		// (b) dimension;
		read_dimension(mod,scale);

		// (c) pins of this module;	  
		read_IO_list(mod,final);

		// (d) if this was the last module, which is the "TYPE PARENT;" one,
		// then read in one shot only the actual network;
		if (final) {
			read_network();
			break;
		}
	}
	


	// (2) part 2: read communication volumes for each net;
	// these volumes should be listed in the order of arcs from the
	// communication task graph;
	fs >> t1 >> t2;
	// sanity check;
	if ( strcmp(t1,"COMMUNICATIONS")) {
		// input file is corrupted;
		printf("\nError: Input file should contain COMMUNICATIONS\n");
		exit(1); 
	}

	// Note: nets n0,n1,n2,...etc. are recorded at net index starting at 3,
	// because net indices 0,1,2 are used first time inside read_IO_list()
	// for the VSS,VDD,GND; so, handle this with care when using net 
	// indices: do not forget about this bias of 3...;
	// net_table.size() = num. actual nets + the nets from the PARENT module;
	_communications_volume.resize( net_table.size());
	_edge_src.resize( net_table.size());
	_edge_des.resize( net_table.size());
	for (int i=0; i < net_table.size(); i++) {
		_communications_volume[i] = 1;
		_edge_src[i] = -1;
		_edge_des[i] = -1;		
	}

	for ( int i = 0; !fs.eof(); i++) {
		int src, des, volume;
		fs >> t1 >> src >> des >> volume;
		if ( !strcmp(t1,"END;"))
			break; // done;
		// get the actual net id (this is the real net id stored in all pins
		// inside modules);
		int net_id = net_table[t1]; // t1 = n17 for example;
		_edge_src[net_id] = src;
		_edge_des[net_id] = des;		
		_communications_volume[net_id] = volume;
		//printf("\n volume: %d", volume);
	}


	
	// (3) create the actual list of nets; a net is an entry of network as
	// a list of pins of various modules;
	root_module = modules.back();
	modules.pop_back(); // exclude the parent module;
	modules_N = modules.size();	 
	modules_info.resize(modules_N);
	modules.resize(modules_N);

	// () create the actual list of nets as arrays of pin pointers; here we
	// also create nets for example for the 3 VSS,VDD,GND due to the last 
	// parent module (which itself is removed from the list of modules);
	// these 3 nets VSS,VDD,GND are global nets; should not be used in any 
	// of the subfloorplans;
	create_network();

	TotalArea = 0;
	for (int i=0; i < modules_N; i++) {
		TotalArea += modules[i].area;
	}
}

void FPlan::read_dimension(Module &mod,float scale)
{
	// read dimensions of this module;
	// example:
	// DIMENSIONS 3037 0 3037 1826 -109 1826 -109 0;

	fs >> t1;
	int min_x=INT_MAX,min_y=INT_MAX,max_x=INT_MIN,max_y=INT_MIN;
	int tx,ty;
	for(int i=0; i < 4;i++){
		fs >> tx >> ty; 
		// apply a scale multiplier (rounding numbers with +0.5)
		if (scale!=1.0) {
			tx=(int) (tx*scale+0.5);
			ty=(int) (ty*scale+0.5);
		}
		min_x=min(min_x,tx); max_x=max(max_x,tx);
		min_y=min(min_y,ty); max_y=max(max_y,ty);
	}

	mod.x	   = min_x;
	mod.y	   = min_y;
	mod.width  = max_x - min_x;
	mod.height = max_y - min_y;
	mod.area   = mod.width * mod.height;
	fs >> t1 >> t2;
}

void FPlan::read_IO_list(Module &mod,bool parent=false)
{
	// read IO pins of this module: create its list of Pins;
	// example:
	// IOLIST;
	//  P_0 B 2 0 1 METAL2;
	//  P_1 B 42 0 1 METAL2;
	//  ...
	//  P_4 B 58 0 1 METAL2;
	// ENDIOLIST;

	while (!fs.eof()){
		Pin p;
		fs.getline(line,100);
		if (strlen(line)==0) continue;
		sscanf(line,"%s %*s %d %d",t1, &p.x, &p.y);

		if (!strcmp(t1, "ENDIOLIST;"))
			break;

		if (parent) {
			// make unique net id;
			// Note: the parent module will have its pins created as the first
			// nets and stored in the net_table; VSS, VDD, GND will be the nets
			// with index 0,1,2;
			net_table.insert(make_pair(string(t1),net_table.size()));
			//printf("\n...Created net  %s  net_id: %d", t1, net_table[t1]);
			p.net = net_table[t1];
		}

		p.mod = mod.id;
		p.x -= mod.x;  p.y -= mod.y; // shift to origin;
		mod.pins.push_back(p); // record this pin to module mod;
	}
	fs.getline(line,100);
}

void FPlan::read_network()
{
	// called only once;
	// example:
	// NETWORK;
	//  c_0 cc_11 n0 n1 n2 n3 n4 n5 n6;
	//  c_1 cc_12 n0 n7 n8 n9 n10 n11 n12;
	//  ...
	//  c_7 cc_24 n6 n12 n17 n21 n24 n26 n27;
	// ENDNETWORK;

	while (!fs.eof()) {
		bool end=false;
		int n = 0;
		fs >> t1 >> t2;
		if(!strcmp(t1,"ENDNETWORK;"))
			break;

		// find the module id this current determine which module interconnection by name
		int m_id;
		for (m_id=0; m_id < modules.size(); m_id++) {
			if(!strcmp(modules[m_id].name,t2)) {
				break;
			}
		}
		if (m_id== modules.size())
			error("Can't find suitable module name!");

		while (!fs.eof()){
			fs >> t1;
			if(t1[strlen(t1)-1]==';'){
				tail(t1);
				end = true;
			}

			// create or retreive unique net id for this net name (e.g., n17);
			net_table.insert(make_pair(string(t1),net_table.size()));
			//printf("\n...Created net  %s  net_id: %d", t1, net_table[t1]);
			// record this unique net id in the pin id "n" of this module;
			// example: modules[0].pins[0,1,2,3,4,5,6].net = 3,4,5,6,7,8,9;
			// net n0 got index 3 (because of the parent module which has created 3 
			// nets) but I declare it later at the end of the testcase such 
			// that its id is 0 and will record is srcand des accordingly;
			modules[m_id].pins[n++].net = net_table[t1];
			if (end) break;
		}
	}
}

void FPlan::create_network()
{

	// (0) part 1: create actual list of nets as lists of pin pointers 
	// (pins that were created during module creation);
	network.resize(net_table.size());

	for (int i=0; i < modules_N; i++){
		for(int j=0; j < modules[i].pins.size(); j++){
			Pin &p = modules[i].pins[j];
			network[p.net].push_back(&p);
		}
	}

	for (int j=0; j < root_module.pins.size(); j++){
		Pin &p = root_module.pins[j];
		network[p.net].push_back(&p);
	}



	// (1) _connection is used only for testcase creation purposes;
	_connection.resize(modules_N+1);
	for (int i=0; i < modules_N+1; i++) {
		_connection[i].resize(modules_N+1);
		fill(_connection[i].begin(), _connection[i].end(), 0);
	}
	for (int i=0; i < network.size(); i++) {
		for (int j=0; j < network[i].size()-1; j++) {
			int p = network[i][j]->mod; // module id of j-th pin of i-th net index;
			for (int k=j+1; k < network[i].size(); k++) {
				int q = network[i][k]->mod; // module id of k-th pin of i-th net index;
				// if p is the source of any arc and q is its destination
				// then record the comm volume as the _connection[p][q];
				bool pq_is_an_arc = false;
				for (int u=0; u < _edge_src.size(); u++) {
					if ( _edge_src[u] == p && _edge_des[u] == q) {
						_connection[p][q] = _communications_volume[u];
					} else if ( _edge_src[u] == q && _edge_des[u] == p) {
						_connection[q][p] = _communications_volume[u];
					}
				}
			}
		}
	}
	


	// (2) debug: show _connections between modules;
	//print_connections_between_modules();
	




	// (3) part 2: not used during normal runs;
	// next is used only with the secret argument for testcase
	// creation; create test case (files output)
	// must be done in Linux so the text files follow the UNIX standard
	// (google for "ifstream tellg UNIX Windows bug")
	if ( _sfra->testcase_creation() == true) {
		double TOTAL_CYCLES_COUNT = 1000000.0; // initially was 10000.0;
		vector<INJECTION > injections;
		INJECTION temp_inj;
		long int total=0,total_this_src=0;

		// multiplier is used to create more packets; this is taken as
		// an argument: start with 1, generate traces and simulate; this
		// process is repeated until we get traces that saturate the NoC
		// at around 50%; be carefull not to overflow variables
		float multiplier;
		
		multiplier = _sfra->testcase_multiplier();
		srand(1);
		vector<pair<int,int > > printed;
		pair<int,int> temp;

		// (a) generate all the injections for all sources;
		for ( int i=0; i < modules_N; i++) {
			for ( int j=0; j < modules_N; j++) {
				if ( i != j && _connection[i][j] > 0) {
					int biased_connection_size = int(_connection[i][j] * multiplier);
					for ( int m=0; m < biased_connection_size; m++) {

						temp_inj._t = // injection time;
							( (double)rand()/(double)RAND_MAX ) * TOTAL_CYCLES_COUNT;
						temp_inj._src = i; // origin;
						temp_inj._des = j; // destination;
						//temp_inj.erased = false;
						injections.push_back( temp_inj);
						total ++;
					}
				}
			}
		}
		printf("\nTotal injection packets:%d\n",total);
		// now sort all the injections by their time of injection;
		sort( injections.begin(), injections.end());

		// (b) create individual trace files with all injections sorted in time;
		FILE *pFile;
		char name[256],xname[253],yname[253],ipname[5];
		// get the name of the .fp file
		strcpy(name, filename.c_str());
		// remove the .fp extension
		strncpy(xname,name,strlen(name)-2);
		xname[strlen(name)-2]='\0';

		printf("\n\nIndividual files:\n\n");
		for ( int k=0; k<modules_N; k++) {
			// "k" is the source processed this iter;
			strcpy(yname,xname);
			// append source index "k" and .trs extensions;
			sprintf(ipname,"%d",k);
			strcat(yname,ipname);
			strcat(yname,".trs");
			pFile = fopen(yname,"w");

			total_this_src = 0;
			for ( int i=0; i<total; i++) {
				if ( injections[i]._src == k) {
					fprintf(pFile,"%f %d %d 5\n", injections[i]._t, k, injections[i]._des);
					total_this_src ++;
				}
			}
			printf("\n\nCreated '%s': (injection packets:%d)\n\n",yname,total_this_src);

			fclose(pFile);
		}

		// (c) create an empty file for the unassigned routers
		strcpy(yname,xname);
		strcat(yname,"-1.trs");
		pFile = fopen(yname,"w");
		fclose(pFile);

		// (d) create the main trace file: includes all injection packets;
		// why is this needed?
		strcat(xname,"trs");
		pFile = fopen(xname,"w");

		for ( int i=0; i<total; i++) {
			fprintf(pFile,"%f %d %d 5\n", injections[i]._t, 
				injections[i]._src, injections[i]._des);
		}
		fclose(pFile);
		printf("\n\nCreated top-level trace file '%s': (injection packets:%d)\n\n",xname,total);

		exit(1);
	}

}

void FPlan::print_connections_between_modules()
{
	vector< pair<int, int> > printed;
	pair<int, int> temp;
	int already_printed;
	// will be used to index "n*" arcs to be added to each .fp testcase;
	int n_index=0;
	int already_n_index=-1; 
	vector<int> sources;
	vector<int> destinations;
	
	// ()
	printf("\n");
	for (int i=0; i < modules_N; i++) {
		printf("\nc_%d %s", i, modules[i].name);
		for (int j=0; j < modules_N; j++){

			already_printed = false;
			for (int k=0; k<printed.size(); k++) {
				if ( (printed[k].first == i && printed[k].second == j) ||
					 (printed[k].first == j && printed[k].second == i)) {
					already_printed = true;
					already_n_index = k;
				}
			}

			if ( i != j && _connection[i][j] > 0) {
				if ( already_printed == false) {
					//printf("%s <-> %s  %d\n", modules[i].name, modules[j].name, _connection[i][j]);
					printf(" n%d", n_index);
					n_index ++;
					sources.push_back( i);
					destinations.push_back( j);

					temp.first = i;
					temp.second = j;				
					printed.push_back(temp);
				} else {
					printf(" n%d", already_n_index);
				}
			}
		}
	}
	printf("\n\n");
	for (int i=0; i < printed.size(); i++) {
		printf("\nn%d %d %d %d", i, sources[i], destinations[i],
			_connection[ sources[i] ][ destinations[i] ]);
	}
	printf("\n_connection[p][q]:");
	for (int i=0; i < modules_N; i++) {
		printf("\n");
		for (int j=0; j < modules_N; j++) {
			printf(" %d", _connection[i][j]);
		}
	}
	printf("\n\n");	
	exit(1);	
}

////////////////////////////////////////////////////////////////////////////////
//
// Wire Length Estimate
//
////////////////////////////////////////////////////////////////////////////////

void FPlan::scaleIOPad()
{
	float px = Width/float(root_module.width);
	float py = Height/float(root_module.height);
	
	for (int i=0; i < root_module.pins.size(); i++){
		Pin &p = root_module.pins[i];
		p.ax = int(px * p.x);
		p.ay = int(py * p.y);
	}	   
}

double FPlan::calcWireLength( bool include_root_module)
{
	// if include_root_module is false then WL is computed w/o
	// considering nets connected to the root_module; default it's true;

	// (1) if the root_module exists, then scale its own pin locations too;
	if ( root_module.id >= 0) {
		scaleIOPad();
	}

	// (2) accumulate WL;
	WireLength = 0;
	// compute absolute positions for all pins of all modules;
	for (int i=0; i < modules_N; i++) {	
		int mx = modules_info[i].x;
		int my = modules_info[i].y;
		bool rotate = modules_info[i].rotate;
		int w = modules[i].width;

		for (int j=0; j < modules[i].pins.size(); j++) {
			Pin &p = modules[i].pins[j];

			if (!rotate) {	  
				p.ax = p.x+mx; p.ay = p.y+my;
			} else { // Y' = W - X, X' = Y
				p.ax = p.y+mx; p.ay = (w-p.x)+my;
			} 
		}
	}

	for (int i=0; i < network.size(); i++) {	   
		int max_x= INT_MIN, max_y= INT_MIN;		 
		int min_x= INT_MAX, min_y= INT_MAX;		 

		assert( network[i].size() > 0);
		bool is_root_module_net = false;
		for (int j=0; j < network[i].size(); j++) {
			Pin &p = *network[i][j];
			//if ( !include_root_module) {
			//	if ( p.mod == root_module.id) {
			//		is_root_module_net = true;
			//		break;
			//	}
			//}
			max_x = max(max_x, p.ax); max_y = max(max_y, p.ay);
			min_x = min(min_x, p.ax); min_y = min(min_y, p.ay);
		}
		//if ( is_root_module_net) continue;
		//printf("%d %d %d %d\n",max_x,min_x,max_y,min_y);
		// Note: here we weight the 1/2 perimeter wirelength of this net
		// with its communication volume;
		WireLength += ( _communications_volume[i] ) * // weight;
			( (max_x-min_x)+(max_y-min_y) );
	}
	return WireLength;
}

void FPlan::get_module_center_coordinates( 
	int mod_i, int &pin_ax, int &pin_ay)
{
	assert( mod_i >= 0 && mod_i < modules_N);

	pin_ax = (modules_info[ mod_i].x + modules_info[ mod_i].rx) / 2;
	pin_ay = (modules_info[ mod_i].y + modules_info[ mod_i].ry) / 2;
}

////////////////////////////////////////////////////////////////////////////////
//
// debug
//
////////////////////////////////////////////////////////////////////////////////

void FPlan::show_modules()
{
	cout << endl;
	for (int i=0; i < modules.size(); i++) {
		cout << "Module " << i << ": " << modules[i].name << endl;
		cout << "  Width = " << modules[i].width;
		cout << "  Height= " << modules[i].height << endl;
		cout << "  Area  = " << modules[i].area << endl;
		
		cout << "  Pins (" << modules[i].pins.size() << "), nets: ";;
		for (int j=0; j < modules[i].pins.size(); j++) {
			cout << " " << modules[i].pins[j].net;
			//cout << modules[i].pins[j].x << " " << modules[i].pins[j].y << endl;
		}
		cout << endl;	
	}
}

void FPlan::list_information()
{
	// removed .info file output
	/*---
	string info = filename + ".info";	  
	ofstream of(info.c_str());
	of << modules_N << " " << Width << " " << Height << endl;
	for(int i=0; i < modules_N; i++){
		of << modules_info[i].x  << " " << modules_info[i].rx	 << " ";
		of << modules_info[i].y << " " << modules_info[i].ry << endl;
	}
	of << endl;
	calcWireLength(); 
	int x,y,rx,ry;
	for(int i=0; i < network.size(); i++){
		assert(network[i].size()>0);
		x = network[i][0]->ax;
		y = network[i][0]->ay;
	
		for(int j=1; j < network[i].size(); j++){
			rx = network[i][j]->ax;
			ry = network[i][j]->ay;
			of << x << " " << y << " " << rx << " " << ry << endl;
			x = rx, y = ry;
		}
	}
	---*/

	cout << "Num of Module  = " << modules_N << endl;
	cout << "Height         = " << Height << endl;
	cout << "Width          = " << Width << endl;
	cout << "Aspect ratio   = " << ((Height >= Width)?(Height/Width):(Width/Height)) << endl;
	cout << "Area (H*W)     = " << Area << endl;
	cout << "Wire Length    = " << calcWireLength( true) << endl;
	cout << "Used Area      = " << TotalArea << endl;
	printf( "Dead Space (%) = %.2f\n", getDeadSpace());
}

////////////////////////////////////////////////////////////////////////////////
//
// Auxilliary Functions
//
////////////////////////////////////////////////////////////////////////////////

void error(char *msg,char *msg2)
{
	printf(msg,msg2);
	cout << endl;
	throw 1;
}

bool rand_bool()
{
	return bool(rand()%2);
}

float rand_01()
{
	return float(rand()%10000)/10000;
}

////////////////////////////////////////////////////////////////////////////////
//
// subfloorplans
// functions related to creating sub-floorplans from a top level
// floorplan which has been partitioned and has modules in partition 0 or 1;
// partition_id dictates what IP/cores are to form the sub-floorplan;
//
////////////////////////////////////////////////////////////////////////////////

void FPlan::create_subfloorplan( int sub_floorplan_id, FPlan *fp_p, 
	vector<int> partition) 
{

	Module dummy_mod;


	// (1) create the magic maps between module ids in top-level and ids
	// in subfloorplan; let's say that in the top level we have 10
	// and this subfloorplan has 5 cores out of the 10;
	// topid_2_subid = {0 -1 -1 1 2 -1 -1 -1 3 4}; top-level node
	// id 9 corresponds to subfloorplan node id 4;
	vector<int> topid_2_subid;
	// subid_2_topid = {0 3 4 8 9};
	vector<int> subid_2_topid;

	int t_modules_N = fp_p->modules_N;
	int counter_i = 0;
	for ( int i=0; i < t_modules_N; i++) {
		topid_2_subid.push_back( -1);
		// if the top level module is in partition with id of the 
		// subfloorplan, then add it to this subfloorplan;
		if ( partition[i] == sub_floorplan_id) {
			// record the magic numbers;
			topid_2_subid[ i] = counter_i;
			subid_2_topid.push_back( i);
			counter_i ++; // prepare for next subfloorplan core;
		}
	}
	// a temporary array for recording if a top-level net is preserved
	// inside the subfloorplan or not;
	int t_network_N = fp_p->network.size();
	vector<int> top_net_preserved;
	for ( int i=0; i < t_network_N; i++) {
		top_net_preserved.push_back( -1);
	}
	

	// (2) create subfloorplan modules; populate them with info from
	// thrir top-level counterparts;
	counter_i = 0;
	for ( int i=0; i < t_modules_N; i++) {

		if ( partition[ i] == sub_floorplan_id) {

			// () create module of subfloorplan;
			modules.push_back( dummy_mod);
			//printf("\nCreated module: %d", counter_i);
			Module &mod = modules.back();
			mod.id = counter_i;
			mod.pins.clear();
			strcpy( mod.name, fp_p->modules[i].name);

			// () copy over its dimensions from the top-level module;
			mod.x	   = fp_p->modules[i].x;
			mod.y	   = fp_p->modules[i].y;
			mod.width  = fp_p->modules[i].width;
			mod.height = fp_p->modules[i].height;
			mod.area   = fp_p->modules[i].area;

			// () copy over only the pins of nets within the partition
			// with the corrected ids according to the magic numbers;
			//printf("\nModule: %d", i);
			for (int j=0; j < fp_p->modules[i].pins.size(); j++) {
				Pin &t_p = fp_p->modules[i].pins[j];
				int net_id = fp_p->modules[i].pins[j].net;
				// if this top-level net has the other terminal (all nets
				// in the appl. task graph have only two terminals, src and dest),
				// in the other partition then, we do not copy this pin; the
				// pin should be removed because it will not form a new net inside
				// the subfloorplan;
				int src_id = fp_p->_edge_src[ net_id];
				int des_id = fp_p->_edge_des[ net_id];
				//printf("\n net_id: %d src: %d des: %d", net_id, src_id,des_id);
				assert( i == src_id || i == des_id);
				int other_i = (i == src_id) ? des_id : src_id;
				if ( partition[ other_i] != sub_floorplan_id) {
					// skip this pin altogether because the othe pin is in the 
					// other partition;
					//cout << " NOT Within same part: " << fp_p->modules[other_i].name;
					continue;
				}
				// if here, the other pin is within the same partition, so we copy 
				// the pin to the module in the subfloorplan;
				//cout << " Within same part: " << fp_p->modules[other_i].name;
				Pin p;
				p.mod = counter_i;
				p.x = t_p.x; p.y = t_p.y;
				// the net from the top-level floorplan will be mapped to a subfloorplan
				// net with new index with subfloorplan; create unique ids here same
				// way we do when we read from file the top-level floorplan;
				// Note: i do not like this my_ss technique; i should really change it
				// to something more elegant;
				stringstream my_ss;
				my_ss << net_id;
				net_table.insert(make_pair(my_ss.str(), net_table.size()));
				//printf("\nCreated subfloorplan net id: %d", p.net);
				p.net = net_table[ my_ss.str()];
				top_net_preserved[ net_id] = p.net; // (re)record for later use;
				mod.pins.push_back( p); // record this new pin to subfloorplan module mod;
			} // for pins

			counter_i ++; // prepare for next subfloorplan module;
		}
	}	


	// (3) done creating modules of subfloorplan with only the pins of
	// within partition nets; new subfloorplan net ids were also created;
	// now, we copy for these subfloorplan nets their comm. volume;
	_communications_volume.resize( net_table.size());
	for ( int i=0; i < t_network_N; i++) {
		if ( top_net_preserved[ i] >= 0) {
			stringstream my_ss;
			my_ss << i;
			int sub_fp_net_id = net_table[ my_ss.str()]; 
			_communications_volume[sub_fp_net_id] = fp_p->communications_volume(i);
			//printf("\ncomm_volume[%d]: %d",sub_fp_net_id,_communications_volume[sub_fp_net_id]);
		}
	}


	// (4) create network of subfloorplan;
	// create actual list of nets as lists of pin pointers 
	// (pins that were created during module creation);
	modules_N = modules.size(); // record final number of modules of subfloorplan;
	modules_info.resize(modules_N); // allocate mem for modules_info - used during SA;
	network.resize(net_table.size());
	for (int i=0; i < modules_N; i++){
		for(int j=0; j < modules[i].pins.size(); j++){
			Pin &p = modules[i].pins[j];
			network[p.net].push_back(&p);
		}
	}


	TotalArea = 0;
	for (int i=0; i < modules_N; i++) {
		TotalArea += modules[i].area;
	}


	// () debug;
	//show_modules();
}

void FPlan::create_subfloorplan_layer2(
	FPlan *fp_toplevel, FPlan *fp_p1, vector<int> partition)	
{
	Module dummy_mod;


	// (1) create the magic maps between module ids in top-level and ids
	// in subfloorplan; let's say that in the top level we have 10
	// and this subfloorplan has 5 cores out of the 10;
	// topid_tp_subid = {0 -1 -1 1 2 -1 -1 -1 3 4}; top-level node
	// id 9 corresponds to subfloorplan node id 4;
	vector<int> topid_to_subid_1;
	// subid_to_topid = {0 3 4 8 9};
	vector<int> subid_to_topid_1;

	vector<int> topid_to_subid_2;
	vector<int> subid_to_topid_2;

	int modules_N_toplevel = fp_toplevel->modules_N;
	int counter_i_1 = 0;
	int counter_i_2 = 0;
	for ( int i=0; i < modules_N_toplevel; i++) {
		topid_to_subid_1.push_back( -1);
		topid_to_subid_2.push_back( -1);
		// if the top level module is in partition with id of the 
		// subfloorplan, then add it to this subfloorplan;
		if ( partition[i] == 0) {
			// record the magic numbers;
			topid_to_subid_1[ i] = counter_i_1;
			subid_to_topid_1.push_back( i);
			counter_i_1 ++;
		} else if ( partition[i] == 1) {
			topid_to_subid_2[ i] = counter_i_2;
			subid_to_topid_2.push_back( i);
			counter_i_2 ++;
		}
	}
	int modules_N_1 = counter_i_1;
	int modules_N_2 = counter_i_2;
	// a temporary array for recording if a top-level net is preserved
	// inside the subfloorplan 2, on layer 2, or not;
	int network_N_toplevel = fp_toplevel->network.size();
	vector<int> top_net_preserved;
	for ( int i=0; i < network_N_toplevel; i++) {
		top_net_preserved.push_back( -1);
	}
	


	// (2) create subfloorplan 2 modules; populate them with info from
	// their top-level counterparts;

	// first create the root_module, which will have toplevel nets connected
	// to the modules of subfloorplan 2; these nets are the nets with one pin
	// attached to modeules of subfloorplan 2 and the other pin - fixed - 
	// corresponding to modules of subfloorplan 1, which has been done already;
	// this way we take into accout the locations of the already floorplanned
	// layer 1;
	// keep its id as -1 instead of modules_N_2 in order to disable 
	// the use of scaleIOPad() inside calcWireLength();
	root_module.id = -1; 
	root_module.x = 0;
	root_module.y = 0;	

	// now add the actual modules in partition "1";
	counter_i_2 = 0;
	for ( int i=0; i < modules_N_toplevel; i++) {

		if ( partition[ i] == 1) {

			// () create module of subfloorplan;
			modules.push_back( dummy_mod);
			//printf("\nCreated module: %d", counter_i_2);
			Module &mod = modules.back();
			mod.id = counter_i_2;
			mod.pins.clear();
			strcpy( mod.name, fp_toplevel->modules[i].name);

			// () copy over its dimensions from the top-level module;
			mod.x	   = fp_toplevel->modules[i].x;
			mod.y	   = fp_toplevel->modules[i].y;
			mod.width  = fp_toplevel->modules[i].width;
			mod.height = fp_toplevel->modules[i].height;
			mod.area   = fp_toplevel->modules[i].area;

			// () copy over only all pins of nets within or not the partition
			// with the corrected ids according to the magic numbers;
			//printf("\nModule: %d", i);
			for (int j=0; j < fp_toplevel->modules[i].pins.size(); j++) {
				Pin &p_toplevel = fp_toplevel->modules[i].pins[j];
				int net_id = fp_toplevel->modules[i].pins[j].net;
				// if this top-level net has the other terminal (all nets
				// in the appl. task graph have only two terminals, src and dest),
				// in the other partition then, we copy the other pin to root_module;
				// it will act as a fixed pin for this net which is still part
				// of the subfloorplan 2;
				int src_id = fp_toplevel->_edge_src[ net_id];
				int des_id = fp_toplevel->_edge_des[ net_id];
				//printf("\n net_id: %d src: %d des: %d", net_id, src_id,des_id);
				assert( i == src_id || i == des_id);
				int other_i = (i == src_id) ? des_id : src_id;


				// here, we copy every pin anyway, irrespective of if the other
				// pin of the net is in same partition or not;
				Pin p;
				p.mod = counter_i_2;
				p.x = p_toplevel.x; p.y = p_toplevel.y;
				// the net from the top-level floorplan will be mapped to a subfloorplan
				// net with new index with subfloorplan; create unique ids here same
				// way we do when we read from file the top-level floorplan;
				// Note: i do not like this my_ss technique; i should really change it
				// to something more elegant;
				stringstream my_ss;
				my_ss << net_id;
				net_table.insert(make_pair(my_ss.str(), net_table.size()));
				//printf("\nCreated subfloorplan net id: %d", p.net);
				p.net = net_table[ my_ss.str()];
				top_net_preserved[ net_id] = p.net; // (re)record for later use;
				mod.pins.push_back( p); // record new pin to subfloorplan 2 module mod;


				if ( partition[ other_i] != 1) {
					// the other pin is of a module from subfloorplan 1 on layer 1;
					// we add it here as a pin of PARENT (ie root_module) of this
					// subfloorplan 2; this pin has fixed coordinates and will act
					// as an anchor during SA; the net from the toplevel is also 
					// preserved, but this net will have one pin fixed;
					//cout << " NOT Within same part: " << fp_toplevel->modules[other_i].name;
					Pin p_fixed;
					p_fixed.mod = -1; // root_module is kept with -1 as id;

					// get center of module from subfloorplan 1 and use as coordinates
					// of fixed pin for now;
					int pin_ax = 0, pin_ay = 0;
					int mod_i = topid_to_subid_1[ other_i];
					fp_p1->get_module_center_coordinates( mod_i, pin_ax, pin_ay);
					p_fixed.x = pin_ax;
					p_fixed.y = pin_ay;
					p_fixed.ax = pin_ax;
					p_fixed.ay = pin_ay;
					p_fixed.net = net_table[ my_ss.str()];
					root_module.pins.push_back( p_fixed); // record this new pin to subfloorplan module mod;
				}
			} // for pins

			counter_i_2 ++; // prepare for next subfloorplan module;
		}
	}	


	// (3) done creating modules of subfloorplan 2 with all pins;
	// new subfloorplan 2 net-ids were also created;
	// now, we copy for these subfloorplan nets their comm. volume;
	_communications_volume.resize( net_table.size());
	for ( int i=0; i < network_N_toplevel; i++) {
		if ( top_net_preserved[ i] >= 0) {
			stringstream my_ss;
			my_ss << i;
			int sub_fp_net_id = net_table[ my_ss.str()]; 
			_communications_volume[sub_fp_net_id] = fp_toplevel->communications_volume(i);
			//printf("\ncomm_volume[%d]: %d",sub_fp_net_id,_communications_volume[sub_fp_net_id]);
		}
	}


	// (4) create network of subfloorplan;
	// create actual list of nets as lists of pin pointers 
	// (pins that were created during module creation);
	modules_N = modules.size(); // record final number of modules of subfloorplan;
	modules_info.resize(modules_N); // allocate mem for modules_info - used during SA;
	network.resize(net_table.size());
	for (int i=0; i < modules_N; i++) {
		for (int j=0; j < modules[i].pins.size(); j++) {
			Pin &p = modules[i].pins[j];
			network[p.net].push_back( &p);
		}
	}
	// add also the pins of the root_module with all its pins fixed;
	for (int j=0; j < root_module.pins.size(); j++) {
		Pin &p = root_module.pins[j];
		network[p.net].push_back( &p);
	}


	TotalArea = 0;
	for (int i=0; i < modules_N; i++) {
		TotalArea += modules[i].area;
	}


	// () debug;
	//show_modules();
}
