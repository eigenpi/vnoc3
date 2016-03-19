#ifndef _HMETIS_INTERFACE_
#define _HMETIS_INTERFACE_

// asta este o librarie privata. O am de la Wonjoon; merge cu arrays
// cu floats in loc de intregi; e diferit de hMetis care este online;
// am schimbat interfata clasica cu una noua:
// void HMETIS_PartRecursive(int, int, float*, int*, int*, float*,
//                           int, float, int*, int*, float*);
//
// hMetis is a hyper-graph partitioner that sees
// vertices & hyper-edges. In VLSI problems, we 
// have modules, pads and multi-point nets connecting 
// them. The interface class MetisIntfc has been written
// such that you don't have to bother much about
// the internal representation of vertices, hyper-edges, etc.
// 
// Instead, you will only think about:
// - Modules
// - Pads
// - Multi-point Nets
// Also, you do not have to worry about Modules, Pads, 
// Nets, etc. being numbered from 0..n, etc.
// As long as each of the Modules has a unique Integer ID,
// each of the Nets has a unique Integer NetID and each
// of the Pads has a unique Integer Pad ID, you are fine.
// 
// To use hMetis, you will do the following
// - Set up the Circuit Netlist
// - First tell hmetis what the Modules & Pads are
// - Then insert each of the nets - one net at a time
// - Set up the initial partition information
// - You can assign partitions to Modules / Pads
// - Or you can keep them movable (unknown partition)
// hMetis will find the paritions for those objects
// - Set up the weights for Modules / Pads / Nets
// - Set up hMetis-specific Options (see comments below
// for what each of these options is);
// - Run the Partitioner
// - Read information about each of the module locations

#include "assert.h"
#include <vector>
#include <map>

using namespace std;

#define DEBUG 1
#define METIS_UNIQUE_NUMBER 888 // check if a module has already been inserted;

extern "C" 
{
	// new:
	void  HMETIS_PartRecursive(int nvtxs, int nhedges, float *vwgts, 
							   int *eptr, int *eind, float *hewgts, 
							   int nparts, float ubfactor, 
							   int *options, int *part, float *edgecut);
	// old:
	// void HMETIS_PartRecursive(int nvtxs, int nhedges, int *vwgts, 
	//					  int *eptr, int *eind, int *hewgts, 
	//					  int nparts, int ubfactor, 
	//					  int *options, int *part, int *edgecut);
};

typedef std::map<int,int> IntMapType;

class MetisIntfc
{
 private:
	// --- hMetis data;
	int nvtxs_, nhedges_;   
	// number of vertices and the number of hyperedges in the hypergraph;
	vector<int> eptr_; // size: nhedges + 1 
	vector<int> eind_;
	//  eptr, eind 
	//    Two arrays that are used to describe the hyperedges in the graph. 
	//    The first array, eptr, is of size nhedges+1, and it is used to index 
	//    the second array eind that stores the actual hyperedges. Each hyperedge 
	//    is stored as a sequence of the vertices that it spans, in consecutive 
	//    locations in eind. Specifically, the i-th hyperedge is stored starting 
	//    at location eind[eptr[i]] up to (but not including) eind[eptr[i + 1]]. 
	//    Figure 6 illustrates this format for a simple hypergraph. The size of 
	//    the array eind depends on the number and type of hyperedges. Also note 
	//    that the numbering of vertices starts from 0.
	// vector<int> vwgts_; // old; size: nvtxs 
	vector<float> vwgts_; // new;
	//  vwgts     
	//    An array of size nvtxs that stores the weight of the vertices.
	//    Specifically, the weight of vertex i is stored at vwgts[i]. If the 
	//    vertices in the hypergraph are unweighted, then vwgts can be NULL.
	// vector<int> hewgts_; // old; size nhedges 
	vector<float> hewgts_; // new;
	//  hewgts   
	//    An array of size nhedges that stores the weight of the hyperedges. 
	//    The weight of the i hyperedge is stored at location hewgts[i]. If the 
	//    hyperedges in the hypergraph are unweighted, then hewgts can be NULL.
	int nparts_; // number of desired partitions;
	vector<int> part_; // number of the partition 0,1..
	//  part     
	//    This is an array of size nvtxs that returns the computed partition. 
	//    Specifically, part[i] contains the partition 
	//    number in which vertex i belongs to. Note that partition numbers start from 0. 
	//    Note that if options[6] = 1, then the initial values of part are used 
	//    to specify the vertex pre­assignment 
	//    requirements.
	// int edgecut_; // old;
	float edgecut_; // new;
	//  edgecut
	//    This is an integer that returns the number of hyperedges that are 
	//    being cut by the partitioning algorithm.
	// Options (initialize to defaults)
	// int ubfactor_; // old;
	float ubfactor_; // new;
	vector<int> options_; // size of 9

	// --- data used to set-up the hyper-graph;
	int net_counter;
	int eind_counter;
	int mod_counter;

	// --- netlist specific data;
	// modules will be numbered internally from 0 .. numMods and 
	// 0 .. numPads and 0..numNets
	int numMods; // init to 0;
	int numNets;
	// note: user specifies an Id (or a name)
	IntMapType modId2vertexNum; // from user-specified id to graph index;
	IntMapType netId2edgeNum;
	IntMapType modAlreadyPresent; // keeps track of insrted module numbers;
	IntMapType netAlreadyPresent;
	int ModIdxOffset;
	// vertices are numbered as 0...NumMods-1 (modules)
	// NumMods .. NumMods+NumPads-1 (pads)
	// so, modVertexNum = ModIdx + ModIdxOffset; padVertexNum = PadIdx + PadIdxOffset;  
	inline int ModVtxNum( int ModIdx );
	int *CreateIntArray(const vector<int> &input);
	void DeleteIntArray(int *int_array);
	float *CreateFloatArray(const vector<float> & input);
	void DeleteFloatArray(float *float_array);

 public:
	MetisIntfc(int num_modules, int num_nets);
	~MetisIntfc() {};

	// setting various options
	int SetDefaults(void);
	int SetUBFactor(const char* optionName, float optionValue);
	int SetOption(const char* optionName, int optionValue);
	// building the Hyper-Graph
	void AddModule(int modIdNumber);
	void AddNet(int netIdNumber, int numModsInNet, const int * Mods); // new version
	// setting up the Problem
	void setModPartition(int modIdNumber, int partitionNumber);
	// void setModWeight(int modIdNumber, int weight); // old;
	void setModWeight(int modIdNumber, float weight); // new;
	// void setNetWeight(int netIdNumber, int weight); // old;
	void setNetWeight(int netIdNumber, float weight);
	int Partition(int numPartitions); // the real call to hMetis partitioner!
	// viewing the results
	int getModPartition(int modIdNumber);
	// debug;
	void ShowModuleInfo(int modIdNumber);
	void ShowNetInfo(int netIdNumber);
	void ShowHyperGraph(void);
	void ShowOptions(void);
	void ShowResults(void);
	int CutSize(void);
	int EvaluateBipartitionBasedCuts(void);
};

// detailed description of the whole thing;
// Options that can be set in the HMetis wrapper & their default values
// To set these, use SetOption("OptionName", option_value);
//
// ---> OptionName: UBfactor
// 
// This parameter is used to specify the allowed imbalance between the 
// partitions during recursive bisection. 
// This is an integer number between 1 and 49, and specifies the allowed
// load imbalance in the following way. Consider a hypergraph with n 
// vertices, each having a unit weight, and let b be the UBfactor. Then, if 
// the number of desired partitions is two (i.e., we perform a bisection), 
// then the number of vertices assigned to each one of the two partitions 
// will be between (50 this allowed imbalance is applied at each bisection 
// step, so if instead of a 2­way partition we are interested in a 4­way 
// partition, then a UBfactor of 5 will result in partitions that 
// can contain between 0.45 2 n = 0.20n and 0.55 2 n = 0.30n vertices. 
// Also note that shmetis does not allow you to produce perfectly balanced 
// partitions. This is a limitation that will be lifted in future releases. 
// 
// ---> OptionName: Nruns
//
// This is the number of the different bisections that 
// are performed by hmetis. It is a number greater or equal 
// to one, and instructs hmetis to compute Nruns different 
// bisections, and select the best as the final solution. 
// A default value of 10 is used by shmetis. 
//
// ---> OptionName: CType
//
// This is the type of vertex grouping scheme (i.e., matching scheme) 
// to use during the coarsening phase. It 
// is an integer parameter and the possible values are: 
//
// 1 Selects the hybrid first­choice scheme (HFC). 
// This scheme is a combination of the first­choice and 
// greedy first­choice scheme described later. This is 
// DEFAULT used by shmetis
//
// 2 Selects the first­choice scheme (FC). In this scheme vertices 
// are grouped together if they are present in 
// multiple hyperedges. Groups of vertices of arbitrary size are 
// allowed to be collapsed together. 
// 
// 3 Selects the greedy first­choice scheme (GFC). In this scheme 
// vertices are grouped based on the first­ 
// choice scheme, but the grouping is biased in favor of faster 
// reduction in the number of the hyperedges that remain in the 
// coarse hypergraphs. 
//
// 4 Selects the hyperedge scheme. In this scheme vertices are 
// grouped together that correspond to entire 
// hyperedges. Preference is given to hyperedges that have large weight. 
//
// 5 Selects the edge scheme. In this scheme pairs of vertices are grouped 
// together if they are connected by 
// multiple hyperedges.
// 
// ---> OptionName: RType 
// 
// This is the type of refinement policy to use during the 
// uncoarsening phase. It is an integer parameter and the possible 
// values are: 
//
// 1 Selects the Fiduccia­Mattheyses (FM) refinement scheme. 
// This is the scheme used by shmetis. (DEFAULT)
// 
// 2 Selects the one­way Fiduccia­Mattheyses refinement scheme. 
// In this scheme, during each iteration of 
// the FM algorithm, vertices are allowed to move only in a single direction. 
//
// 3 Selects the early­exit FM refinement scheme. In this scheme, 
// the FM iteration is aborted if the quality 
// of the solution does not improve after a relatively small number 
// of vertex moves. 
// 
// ---> OptionName: Vcycle 
//
// This parameter selects the type of V ­cycle refinement 
// to be used by the algorithm. It is an integer parameter 
// and the possible values are: 
//
// 0 Does not perform any form of V ­cycle refinement. 
//
// 1 Performs V ­cycle refinement on the final solution of 
// each bisection step. That is, only the best of the 
// Nruns bisections are refined using V ­cycles. This is 
// the options used by shmetis. (DEFAULT)
//
// 2 Performs V ­cycle refinement on each intermediate 
// solution whose quality is equally good or better than 
// the best found so far. That is, as hmetis computes Nruns 
// bisections, for each bisection that matches or 
// improves the best one, it is also further refined using V ­cycles. 
//
// 3 Performs V ­cycle refinement on each intermediate solution. T
// hat is, each one of the Nruns bisections 
// is also refined using V ­cycles. 
//
// Experiments have shown that the second and third choices offer 
// the best time/quality tradeoffs. If time is 
// not an issue, the fourth choice (i.e., Vcycle = 3) should be used. 
//
// ---> OptionName: Reconst 
// 
// This parameter is used to select the scheme to be used in dealing 
// with hyperedges that are being cut during the recursive bisection. 
// It is an integer parameter and the possible values are: 
//
// 0 This scheme removes any hyperedges that were cut while 
// constructing the two smaller hypergraphs in 
// the recursive bisection step. In other words, once a hyperedge 
// is being cut, it is removed from further 
// consideration. Essentially this scheme focuses on minimizing 
// the number of hyperedges that are being 
// cut. This is the scheme that is used by shmetis. (DEFAULT)


// 1 This scheme reconstructs the hyperedges that are being cut, 
// so that each of the two partitions retain the 
// portion of the hyperedge that corresponds to its set of vertices. 

// Section 5.2.2 provides an experimental evaluation of the effect 
// of Reconst in the quality of k­way partitionings. 

// OptionName: Fix

// Determines whether or not there are sets of vertices that need 
// to be pre­assigned to certain partitions. A value of 0 indicates 
// that no pre­assignment is desired, whereas a value of 1 
// indicates that there are sets of vertices that need to be pre­assigned. 
// In this later case, the parameter part_ is used to specify
// the partitions to which vertices are pre­assigned. In particular, 
// part_[i] will store the partition number that vertex i is 
// pre­assigned to , and -1 if it is free to move
//
// ---> OptionName: Seed
// 
// Determines the random seed to be used to initialize the random number generator
// of hMETIS// A negative value indicates that a randomly generated seed 
// should be used (default behavior). 
//
// ---> OptionName: dbglvl 
// 
// This is used to request hMETIS to print debugging information. 
// The value of dbglvl is computed as the sum 
// of codes associated with each option of hmetis. 
// The various options and their values are as follows: 
//
// 0 Show no additional information. 
// 1 Show information about the coarsening phase. 
// 2 Show information about the initial partitioning phase. 
// 4 Show information about the refinement phase. 
// 8 Show information about the multiple runs. 
// 16 Show additional information about the multiple runs. 

#endif
