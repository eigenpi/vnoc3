//---------------------------------------------------------------------------
#ifndef fplanH
#define fplanH
//---------------------------------------------------------------------------
#include <config.h>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <cstdio>

using namespace std;

class SFRA;

//---------------------------------------------------------------------------

struct Pin{
	int mod;
	int net;
	int x,y;	// relative position
	int ax,ay;	// absolute position
	Pin(int x_=-1,int y_=-1){ x=x_,y=y_; }
};
typedef vector<Pin> Pins;
typedef Pin* Pin_p;
typedef vector<Pin_p> Net;
typedef vector<Net > Nets;

enum Module_Type { MT_Hard, MT_Soft, MT_Reclinear, MT_Buffer };

struct Module{
	int id;
	char name[20];
	int width,height;
	int x,y;
	int area;
	Pins pins;
	Module_Type type;
};
typedef vector<Module> Modules;

struct Module_Info{
	bool rotate, flip;
	int x,y;
	int rx,ry;
};

typedef vector<Module_Info> Modules_Info;


////////////////////////////////////////////////////////////////////////////////
//
// FPlan
//
////////////////////////////////////////////////////////////////////////////////

class FPlan {
 protected:
	double Area;
	int WireLength;
	double TotalArea;
	
	Module root_module;
	Nets network;
	double norm_area, norm_wire;
	float cost_alpha;
	vector<vector<int> > _connection;
	// sources and destinations as node indices of tasks from
	// the task communication graph;
	vector<int> _edge_src; // sources of all communication arcs
	vector<int> _edge_des; // destinations
	// communication volume of arcs from the application task graph;
	// will be used as weights during wirelength calculation to encourage
	// heavily communicating modules to be placed closer;
	vector<int> _communications_volume;

 private:
	map<string,int> net_table;
	string filename;

 public:
	SFRA *_sfra;
	// these should not be public; should encapsulate them;
	int modules_N;	  
	Modules_Info modules_info;
	Modules modules;
	double Width,Height;


 public:
	FPlan(float calpha);
	~FPlan();

	SFRA *sfra() { return _sfra; }
	void set_sfra_host(SFRA *sfra) { _sfra = sfra; }
	// function used for inflating IP cores for 2D situations;
	void inflate_core( int id, int routers);
	int get_connection( int i, int j) {
		assert( i >= 0 && i < modules_N);
		assert( j >= 0 && j < modules_N);
		return _connection[i][j];
	}
	int get_module_area( int i) {
		assert( i >= 0 && i < modules_N);
		return modules[i].area;
	}
	int communications_volume( int i) {
		assert( i >= 0 && i < _communications_volume.size());
		return _communications_volume[i];
	}
	
	void read(char*,float);
	virtual void init()		=0;
	virtual void packing();
	virtual void perturb()	=0;	   
	virtual void keep_sol()	=0;
	virtual void keep_best()	=0;
	virtual void recover()	=0;
	virtual void recover_best() =0;
	virtual double getCost();

	// functions related to creating sub-floorplans from 
	// a top level floorplan;
	void create_subfloorplan( int sub_floorplan_id, FPlan *fp_p,
		vector<int> partition);
	void create_subfloorplan_layer2( FPlan *fp_toplevel, FPlan *fp_p1,
		vector<int> partition);

	void get_module_center_coordinates( 
		int mod_i, int &pin_ax, int &pin_ay);

	int	   size()		  { return modules_N; }
	double getTotalArea() { return TotalArea; }
	double getArea()	  { return Area;	  }
	int	   getWireLength(){ return WireLength;}
	double getWidth()	  { return Width;	  }
	double getHeight()	  { return Height;	  }
	float  getDeadSpace();

	// information
	void list_information();
	void show_modules();	
	void normalize_cost(int);
	
 private:
	void read_dimension(Module&,float);
	void read_IO_list(Module&,bool parent);
	void read_network();
	void create_network();	 
	void print_connections_between_modules(); // debug;

 protected:
	void clear();
	double calcWireLength( bool include_root_module = false);
	void scaleIOPad();
};


void error(char *msg,char *msg2="");
bool rand_bool();
float rand_01();
	  
// this class is used only for testcase creation; basically
// for generating all the trace files that contain packets to
// be injected during simulation of the NoC;
class INJECTION {
 public:
	double _t;
	int _src;
	int _des;
 public:
	INJECTION() {};
	~INJECTION() {};

	bool operator==(const INJECTION &a) const { return ( _t == a._t); }
	bool operator!=(const INJECTION &a) const { return ( _t != a._t); }
	bool operator<(const INJECTION &a) const { return ( _t < a._t); }
};

#endif
