#include <iostream>
#include "malloc.h"
#include "stdlib.h"

#include "hmetisInterface.h"

// am schimbat interfata clasica cu asta mai recenta de la Wonjoon;
// void  HMETIS_PartRecursive(int, int, float*, int*, int*, float*,
//                            int, float, int*, int*, float*);
//
// EXPLANATION OF THE HMETIS LIBRARY ARGUMENTS - copied from hMetis manual
// 
// nvtxs, nhedges 
// 
// The number of vertices and the number of hyperedges in the hypergraph,
// respectively. 
// 
// vwgts 
// 
// An array of size nvtxs that stores the weight of the vertices.
// Specifically, the weight of vertex i is stored at vwgts[i]. If the 
// vertices in the hypergraph are unweighted, then vwgts can be NULL. 
// 
// eptr, eind 
// 
// Two arrays that are used to describe the hyperedges in the graph. 
// The first array, eptr, is of size nhedges+1, and it is used to index 
// the second array eind that stores the actual hyperedges. Each hyperedge 
// is stored as a sequence of the vertices that it spans, in consecutive 
// locations in eind. Specifically, the i th hyperedge is stored starting 
// at location eind[eptr[i]] up to (but not including) eind[eptr[i + 1]]. 
// Figure 6 illustrates this format for a simple hypergraph. The size of 
// the array eind depends on the number and type of hyperedges. Also note 
// that the numbering of vertices starts from 0. 
// 
// hewgts 
// 
// An array of size nhedges that stores the weight of the hyperedges. 
// The weight of the i hyperedge is stored at location hewgts[i]. If the 
// hyperedges in the hypergraph are unweighted, then hewgts can be NULL. 
// 
// nparts 
// 
// The number of desired partitions. 
// 
// ubfactor 
//
// This is the relative imbalance factor to be used at each bisection step. 
// Its meaning is identical to the UBfactor parameter of shmetis, and hmetis 
// described in Section 3. 
// 
// options 
// 
// This is an array of 9 integers that is used to pass parameters for the 
// various p hases of the algorithm. If options[0]=0 then default values 
// are used. If options[0]=1, then the remaining elements of options are 
// interpreted as follows: 
// 
// options[1] Determines the number of different bisections that is computed 
// at each bisection step of the 
// algorithm. Its meaning is identical to the Nruns parameter of hmetis 
// (described in Section 3.2). 
// 
// options[2] Determines the scheme to be used for grouping 
// vertices during the coarsening phase. Its 
// meaning is identical to the CType parameter of hmetis 
// (described in Section 3.2). 
// 
// options[3] Determines the scheme to be used for refinement during 
// the uncoarsening phase. Its meaning 
// is identical to the RType parameter of hmetis 
// (described in Section 3.2). 
// 
// options[4] Determines the scheme to be used for V ­cycle refinement. 
// Its meaning is identical to the 
// Vcycle parameter of hmetis (described in Section 3.2). 
// 
// options[5] Determines the scheme to be used for reconstructing 
// hyperedges during recursive bisections. 
// Its meaning is identical to the Reconst parameter of hmetis 
// (described in Section 3.2). 
// 
// options[6] Determines whether or not there are sets of vertices 
// that need to be pre­assigned to certain 
// partitions. A value of 0 indicates that no pre­assignment is desired, 
// whereas a value of 1 
// indicates that there are sets of vertices that need to be pre­assigned. 
// In this later case, the pa­ 
// rameter part is used to specify the partitions to which vertices are 
// pre­assigned. In particular, 
// part[i] will store the partition number that vertex i is pre­assigned 
// to , and -1 if it is free to move
// 
// options[7] Determines the random seed to be used to initialize the 
// random number generator of hMETIS. 
// A negative value indicates that a randomly generated seed should be 
// used (default behavior). 
// 
// options[8] Determines the level of debugging information to be printed 
// by hMETIS. Its meaning is iden­ 
// tical to the dbglvl parameter of hmetis (described in Section 3.2). The 
// default value is 0. 
//
// part 
// 
// This is an array of size nvtxs that returns the computed partition. 
// Specifically, part[i] contains the partition 
// number in which vertex i belongs to. Note that partition numbers start from 0. 
// Note that if options[6] = 1, then the initial values of part are used 
// to specify the vertex pre­assignment 
// requirements. 
// 
// edgecut 
// 
// This is an integer that returns the number of hyperedges that are 
// being cut by the partitioning algorithm. 

// The constructor itself does all the Hyper-Graph Setting stuff
// This is because for this particular problem, the hyper-graph
// does not change whenever this partitioner is called
MetisIntfc::MetisIntfc( int num_modules, int num_nets)
{
	// Initialize hmetis related variables
	ubfactor_ = 0;
	nparts_ = 0;
	options_.resize(9);   
	SetDefaults();
	//ShowOptions(); // should be commented out;
	// get all the numbers right
	numMods = num_modules;
	numNets = num_nets;
	// set up number of vertices
	nvtxs_ = numMods;
	//cout << " v=" << nvtxs_;
	nhedges_ = numNets;
	//cout << " e=" << nhedges_;
  
	// internally keeping track of pad/mod index transformation
	// Note: Vertices are numbered as follows:
	// 0...NumMods-1 (modules) NumMods .. NumMods+NumPads-1 (pads)
	// => modVertexNum = ModIdx + ModIdxOffset 
	// padVertexNum = PadIdx + PadIdxOffset
	ModIdxOffset = 0;
	// data used during mod / pad / net insertion to graph
	eind_counter = 0;
	net_counter = 0;
	mod_counter = 0;

	// set up the Module weights & initial partitions
	for (int i = 1; i <= numMods; i++) {
		part_.push_back(-1); // unknown partition
		vwgts_.push_back(1); // vertex weights == 1 to start with
	}

	// Update the array value in the eptr_ array to for the 
	// first net we insert;
	// array value points to the first location of hyperedge
	eptr_.push_back(eind_counter);
}

void MetisIntfc::AddModule( int modIdNumber)
{
	// add all modules & Pads before you add ANY net
	// just some checking on user input
	if (modAlreadyPresent[modIdNumber] == METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error - Module "<<modIdNumber<<" has already been inserted"<<endl;
		exit(0);
	}
	else
		modAlreadyPresent[modIdNumber] = METIS_UNIQUE_NUMBER;

	modId2vertexNum[modIdNumber] = ModVtxNum(mod_counter);
	mod_counter++;  
}

//void MetisIntfc::AddPad(int padIdNumber) {
//	if (padAlreadyPresent[padIdNumber] == METIS_UNIQUE_NUMBER) {
//	cerr<<"\n Error - Pad "<<padIdNumber<<" has already been inserted"<<endl;
//	exit(0);
//}
//else
//	padAlreadyPresent[padIdNumber] = METIS_UNIQUE_NUMBER;
//if(pad_counter == numPads){
//	cerr<<"\n Error - numPads = "<<numPads<<" Cannot add more than "
//		<<pad_counter<<" Pads"<<endl;
//}
//padId2vertexNum[padIdNumber] = PadVtxNum(pad_counter);
//pad_counter++;
//}

void MetisIntfc::AddNet( int netIdNumber, int numModsInNet, const int *Mods)
{
	// This is where you add a net to the hyper-graph
	// netId - whatever you want to call this net
	// ModIds are the modules it is connected to
	// ModIds[0..numMods-1] should contain the 
	// ids of the modules

	int i=0;
	// Make sure that you insert ALL the modules
	// before you insert ANY net
	if (mod_counter != numMods) {
		cerr<<"\n Error Setting up hMetis Hyper Graph: "<<endl
			<<" NumMods = "<<numMods<<endl
			<<" BUT you inserted only "<<mod_counter<<" Mods"<<endl
			<<" ---- You have to insert ALL Mods before you start inserting Nets --"
			<<endl;
		exit(0);
	}
	if (netAlreadyPresent[netIdNumber] == METIS_UNIQUE_NUMBER){
		cerr<<"\n Error - Net "<<netIdNumber<<" has already been inserted"<<endl;
		exit(0);
	} else {
		netAlreadyPresent[netIdNumber] = METIS_UNIQUE_NUMBER;
	}
	// Store the index of this Net that we are adding
	netId2edgeNum[netIdNumber] = net_counter;
	net_counter++; // simply keeps track of the index of the nets

	hewgts_.push_back(1); // default weight on the nets/hyperedges
 
	// add hyper edge to array;
	// Note: eptr_ already points to the next hyper-edge value;
	// Go through all modules in this net & add it to the hyper-edge array;
	for (i=0; i < numModsInNet; i++) {
		int modIdNumber = Mods[i];
		// sanity check;
		if (modAlreadyPresent[modIdNumber] != METIS_UNIQUE_NUMBER) {
			cerr<<"\n Error - Module "<<modIdNumber
				<<" NOT inserted yet but present in Net "<<netIdNumber<<endl;
			exit(0);
		}
		int moduleNumber = modId2vertexNum[modIdNumber];
		eind_.push_back(moduleNumber); 
		// vertex number will be added at the eind_counter location;
		eind_counter++; 
	}
	// update the array value in the eptr_ array to location of next 
	// edge, if it exists;
	eptr_.push_back(eind_counter); 
	// array value points to the first location of next hyperedge
}

inline int MetisIntfc::ModVtxNum( int ModIdx) {
	if ( !( (ModIdx >=0) && (ModIdx < numMods) )) {
		cout<<"\nERROR in ModVtxNum: ModIdx = "<< ModIdx<<endl;
		exit(0);
	}
	return  ModIdx + ModIdxOffset; 
}

//inline int MetisIntfc::PadVtxNum( int PadIdx ) {
//	if(!( (PadIdx >=0) && (PadIdx < numPads) )){
//	cout<<"\nERROR in PadVtxNum: PadIdx = "<<PadIdx<<endl;
//	exit(0);
//}
//return   PadIdx + PadIdxOffset; 
//}

int MetisIntfc::SetDefaults(void)
{
	// we use options_[0] == 1 and we write in the default array
	// the reason we do this is that we want it to pre-assign 
	// vertices as default;
	options_[0] = 1;
	// These are the default options set; Most of them are the
	// defaults used by shmetis (the stand-alone version)
	SetUBFactor("UBfactor", 1); // Note: shmetis uses 5
	SetOption("Nruns", 10); // 1
	SetOption("CType", 1); // 2 hyperedge scheme
	SetOption("RType", 1); // 3
	SetOption("Vcycle", 1); // 4
	SetOption("Reconst", 0); // 5
	SetOption("Fix", 1);   // 6 pre-assign modules
	SetOption("Seed", -1); // 7 seed for random number generator
	SetOption("dbglvl", 0); // 8 No debug info

	return 1;
}

void MetisIntfc::ShowOptions(void)
{
	cout<<"\n-------- HMetis OPTIONS: --------"
		<<"\nUBfactor="<<ubfactor_;
	cout<<", Nruns="<< options_[1];
	cout<<", CType="<<options_[2];
	cout<<", RType="<<options_[3];
	cout<<", Vcycle="<<options_[4];
	cout<<", Reconst="<<options_[5];
	cout<<", Fix="<<options_[6];
	cout<<", Seed="<< options_[7];
	cout<<", dbglvl="<< options_[8];
}

int MetisIntfc::SetUBFactor( const char* optionName, float optionValue)
{
	// see hmetisInterface.h for explanation of the various options;
	if (strcmp(optionName, "UBfactor") == 0) {
		if ((optionValue <1.0)||(optionValue>49.0)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		ubfactor_ = optionValue;
	}
}

int MetisIntfc::SetOption(const char* optionName, int optionValue)
{
	if (strcmp(optionName, "UBfactor") == 0) {
		if ((optionValue <1)||(optionValue>49)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		ubfactor_ = optionValue;
	}

	else if (strcmp(optionName, "Nruns") == 0) { 
		if (optionValue<1) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[1] = optionValue;
	}
  
	else if (strcmp(optionName, "CType") == 0) {
		if ((optionValue<1)||(optionValue>5)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[2] = optionValue;
	}
  
	else if (strcmp(optionName, "RType") == 0) {
    
		if ((optionValue<1)||(optionValue>3)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[3] = optionValue;
	}
  
	else if (strcmp(optionName, "Vcycle") == 0) {
		if ((optionValue<0)||(optionValue>3)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[4] = optionValue;
	}
  
	else if (strcmp(optionName, "Reconst") == 0) {
		if ((optionValue<0)||(optionValue>1)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[5] = optionValue;
	}
  
	else if (strcmp(optionName, "Fix") == 0) { 
		if ((optionValue<0)||(optionValue>1)) {
			cerr<<"\n Error in MetisIntfc::SetOption optionValue out of range "
				<<optionName<<endl;
			return 0;
		}
		options_[6] = optionValue;
	}
  
	else if (strcmp(optionName, "Seed") == 0) {
		options_[7] = optionValue;
	}
  
	else if (strcmp(optionName, "dbglvl") == 0) {
		options_[8] = optionValue;
	}
  
	else {
		cerr<<"\n Error in MetisIntfc::SetOption, could not find the specified "
			<<"optionName"<<endl;
		return 0;
	}
	return 1;
}

void MetisIntfc::ShowResults(void)
{
	cout << "\n hMetis Result: Number of cut edges =" << edgecut_;
}

int MetisIntfc::CutSize(void)
{
	return int(edgecut_);
}

int MetisIntfc::getModPartition( int modIdNumber)
{
	// sanity check;
	if (modAlreadyPresent[modIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in getModPartition - Module "<<modIdNumber
			<<" NOT an inserted Module "
			<<endl;
		exit(0);
	}
	int modNumber = modId2vertexNum[modIdNumber];
	return part_[modNumber];
}

void MetisIntfc::setModPartition( int modIdNumber, int partitionNumber)
{
	// sanity check;
	if (modAlreadyPresent[modIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in setModPartition - Module "<<modIdNumber
			<<" NOT an inserted Module "
			<<endl;
		exit(0);
	}
	int modNumber = modId2vertexNum[modIdNumber];
	part_[modNumber] = partitionNumber;
}

void MetisIntfc::setModWeight( int modIdNumber, float weight)
{
	// this is the new version, working with float weights;
	// old one, the public version of hMetis works with ints;
	// sanity check;
	if (modAlreadyPresent[modIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in setModPartition - Module "<<modIdNumber
			<<" NOT an inserted Module "
			<<endl;
		exit(0);
	}
	int modNumber = modId2vertexNum[modIdNumber];
	vwgts_[modNumber] = weight;
}

//int MetisIntfc::getPadPartition(int padIdNumber) {
//	if (padAlreadyPresent[padIdNumber] != METIS_UNIQUE_NUMBER) {
//	cerr<<"\n Error in setPadPartition - Pad "<<padIdNumber
//		<<" NOT an inserted Pad "
//		<<endl;
//	exit(0);
//}
//int padNumber = padId2vertexNum[padIdNumber];
//return part_[padNumber];
//}

//void MetisIntfc::setPadPartition(int padIdNumber, int partitionNumber) {
//	if (padAlreadyPresent[padIdNumber] != METIS_UNIQUE_NUMBER) {
//	cerr<<"\n Error in setPadPartition - Pad "<<padIdNumber
//		<<" NOT an inserted Pad "
//		<<endl;
//	exit(0);
//}
//int padNumber = padId2vertexNum[padIdNumber];
//part_[padNumber] = partitionNumber;
//}

//void MetisIntfc::setPadWeight(int padIdNumber, int weight) {
//	if (padAlreadyPresent[padIdNumber] != METIS_UNIQUE_NUMBER) {
//	cerr<<"\n Error in setPadPartition - Pad "<<padIdNumber
//		<<" NOT an inserted Pad "
//		<<endl;
//	exit(0);
//}
//int padNumber = padId2vertexNum[padIdNumber];
//vwgts_[padNumber] = weight;
//}

void MetisIntfc::setNetWeight(int netIdNumber, float weight)
{
	// new version of hMetis working with floats;
	if (netAlreadyPresent[netIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in setNetWeight - Net "<<netIdNumber<<" NOT an inserted Net "
			<<endl;
		exit(0);
	} 
	int netNumber = netId2edgeNum[netIdNumber];
	hewgts_[netNumber] = weight;
}

void MetisIntfc::ShowModuleInfo(int modIdNumber)
{
	// just some checking on user input
	if (modAlreadyPresent[modIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in getModPartition - Module "<<modIdNumber
			<<" NOT an inserted Module "
			<<endl;
		exit(0);
	}
	int modNumber = modId2vertexNum[modIdNumber];
	cout<<" Mod: "<<modIdNumber<<" Vtx-"<<modNumber
		<<" Wt: "<<vwgts_[modNumber]
		<<" Prtn: "<<part_[modNumber];
}

void MetisIntfc::ShowNetInfo(int netIdNumber)
{
	if (netAlreadyPresent[netIdNumber] != METIS_UNIQUE_NUMBER) {
		cerr<<"\n Error in setNetPartition - Net "<<netIdNumber
			<<" NOT an inserted Net "
			<<endl;
		exit(0);
	}
	int netNumber = netId2edgeNum[netIdNumber];
	cout<<" Net: "<<netIdNumber<<" HyperEdge: "<<netNumber
		<<" Wt: "<<hewgts_[netNumber];
}

void MetisIntfc::ShowHyperGraph(void)
{
	// print out stuff using the internal index notation;
	// int i=0, j=0;
	//cout<<"\nDisplaying Hyper Graph "<<endl
	//<<" nvtxs_="<<nvtxs_<<" , nhedges_="<<nhedges_<<endl;
	// showing vertices in Hyper Graph, weights, checking if mods have been
	// numbered correctly
	// checking if partitions have been numbered correctly
	//cout<<" VERTICES --> "<<endl;
	//cout<<" i vwgts part_ "<<endl;
	//for(i=0; i<nvtxs_ ; i++)
	//cout<<i<<". "<<vwgts_[i]<<" "<<part_[i]<<endl;
	// checking hyper edges 
	//cout<<" HYPER EDGES --> "<<endl;
	//cout<<" i hewgt vertices.. "<<endl;
	//for(ii=0; ii<nhedges_; ii++){
	//cout<<ii<<". "<<hewgts_[ii]<<" {";
	//for(j=eptr_[ii]; j<eptr_[ii+1]; j++)
	//cout<<" "<<eind_[j];
	//cout<<" }"<<endl;
	//}
 
	cout<<"\nDisplaying Hyper Graph info ----"
		<<" nvtxs_="<<nvtxs_<<" , nhedges_="<<nhedges_<<endl;
	cout<<" Size of vwgts_="<<vwgts_.size()<<endl;
	cout<<" Size of eptr_="<<eptr_.size()<<endl;
	cout<<" Size of eind_="<<eind_.size()<<endl;
	cout<<" Size of hewgts_="<<hewgts_.size()<<endl;
}

int *MetisIntfc::CreateIntArray(const vector<int> &input)
{
	int i=0;
	int *new_array = new int[input.size()];
	// int* new_array = (int*)malloc((input.size())*sizeof(int));
	for (i=0;  i<input.size(); i++)
		new_array[i] = input[i];
	return new_array;
}

void MetisIntfc::DeleteIntArray(int *int_array)
{
	delete [] int_array;
	// free(int_array);
}

float* MetisIntfc::CreateFloatArray(const vector<float> &input)
{
	int i=0;
	float* new_array = new float[input.size()];
	for (i=0;  i<input.size(); i++)
		new_array[i] = input[i];
	return new_array;
}

void MetisIntfc::DeleteFloatArray(float *float_array)
{
	delete [] float_array;
}

int MetisIntfc::Partition(int numPartitions)
{
	// this is where the actual partitioning is done;
	// Perform a simple check on user's consistency
    // in inserting ALL the nets properly
    // Note: When you insert nets, you already check if the 
    // Mods & Pads have been inserted consistently ...so now only
    // check for the nets
	if (net_counter != numNets) {
		cerr<<"\n Error Setting up hMetis Hyper Graph: "<<endl
			<<" NumNets = "<<numNets<<endl
			<<" BUT you inserted only "<<net_counter<<" nets "<<endl
			<<endl;
		exit(0);
	}

	nparts_ = numPartitions;
	//int *vwgts_array = CreateIntArray(vwgts_);
	float *vwgts_array = CreateFloatArray(vwgts_);
	int *eptr_array = CreateIntArray(eptr_);
	int *eind_array = CreateIntArray(eind_);
	//int *hewgts_array = CreateIntArray(hewgts_);
	float *hewgts_array = CreateFloatArray(hewgts_);
	int *options_array = CreateIntArray(options_);
	int *part_array = CreateIntArray(part_);

	// new (merge bine cu vertices fixate):
	HMETIS_PartRecursive(nvtxs_, nhedges_, vwgts_array, eptr_array, eind_array, 
						 hewgts_array, nparts_, ubfactor_, options_array, 
						 part_array, &edgecut_);
	// old:
	// (int, int, int*, int*, int*, int*, int, int, int*, int*, int*);
	//HMETIS_PartRecursive(nvtxs_, nhedges_, vwgts_array, eptr_array, eind_array, 
	//                     hewgts_array, nparts_, ubfactor_, options_array, 
	//                     part_array, &edgecut_);
  
	// Copy the results back into part_;
	for (int i=0; i<part_.size(); i++) {
		// check to ensure hMetis worked
		if (part_array[i] == -1) return 0;
		part_[i] = part_array[i];
	}
  
	// clean-up stuff;
	//DeleteIntArray(vwgts_array);
	DeleteFloatArray(vwgts_array);
	DeleteIntArray(eptr_array);
	DeleteIntArray(eind_array);
	//DeleteIntArray(hewgts_array);
	DeleteFloatArray(hewgts_array);
	DeleteIntArray(options_array);
	DeleteIntArray(part_array);

	return 1;
}

int MetisIntfc::EvaluateBipartitionBasedCuts(void)
{
	// go through hyper-edges and check if the modules are on different 
	// sides of the partition; if so, they for a cut;
	int cuts = 0;
 
	for (int ii=0; ii<nhedges_; ii++) {
		// for each hyper-edge
		//int hyperEdgeWeight = hewgts_[ii];

		int occupiesLeftPartition = 0;
		int occupiesRightPartition = 0;
		for (int j=eptr_[ii]; j<eptr_[ii+1]; j++) {
			// go through all vertices in the hyper-edge
			int vertexNumber = eind_[j];
			if(part_[vertexNumber] == 0)
				occupiesLeftPartition = 1;
			else if(part_[vertexNumber] == 1)
				occupiesRightPartition = 1;
			else
				cerr<<"\n Should abort:  Error in EvaluatePartitionBasedCuts:"
					<<" No Point using this if partition is unknown ";
		} // we did this for all modules in hyper-edge;
    
		if (occupiesRightPartition && occupiesLeftPartition) {
			cuts++;
		}
	}

	return cuts;
}
