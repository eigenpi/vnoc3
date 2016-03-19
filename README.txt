cristinel.ababei@ndsu.edu
March 2010, Fargo ND


Synopsis
========
This is the VNOC3 project. It represents a framework for studying
3D NoC architectures with only 2 or 3 layers. Please see the reference
below for detailes on these two architectures. The framework is built 
around a light version of VNOC (a cycle-accurate NOC simulator) and
a B*tree floorplanner. It utilizes the hMetis partitioner for the
3-layer architecture. It also includes a GUI and a hidden option, which
can be used to generate new trace-files for possibly new testcases you
may have.

[1] V. de Paulo and C. Ababei, 3D Network-on-Chip architectures using 
homogeneous meshes and heterogeneous floorplans, Hindawi Int. Journal of 
Reconfigurable Computing (IJRC) - Special Issue on Int. Conference on 
Reconfigurable Computing and FPGAs (ReConFig 2009), 2010.


Installation
============
The tool was developed on a Linux machine running Fedora 8. However,
it should be compile-able on Windows too. On linux, first edit the 
Makefile to reflect the location where you want to compile and link. 
Then, just type:
> make


How to use the tool
===================
Type "sfra" at the command prompt to get the help:
> sfra


Testcases
---------
The tool takes as an input a floorplan file .fp which first must
be modified to reflect the communications between modules. See any
of the .fp files under tests/ to see this change. The testcases
under tests/ have been initially part of the B*Tree floorplanning
project/archive.


Details
-------
When a new run is executed, a certain number of annealings will be 
done, and each resulting floorplan will be called an "Attempt". These 
annealings can be done seeking wirelength or area minimization - 
controlled by the "alpha" factor: from 0 to 1 (0 - wirelength
driven, 1 area driven). As these Attempts are processed, they
might (or might not) enter a "bests-list", following a criteria of 
either minimun wirelength or minimum area. In our experiments a value
alpha=0.25 achieved on an average good results.
For those attempts which make it into the bests-list, a router
assignment step and then a VNOC simulation will follow. An "injection- 
load" different than 100% can be chosen via the command line, or a load 
sweep can be done in steps as: 10,20,30...100% (for each of these steps
a VNOC simulation is run). Note: "injection load" in fact means or
translates into a certain "packet injection rate (packets/cycle)".
Also, a buffer-size sweep can be done on top of all the above: these
steps would be repeated for several buffer-sizes: default buffer-size, 
and then 2x, 3x, 4x, 5x as large as the initial buffer-size. 
So, for each "Attempt" there can be performed one simulation (simplest 
run) or 5 simulations (buffer-size sweep only) or 10 simulations 
("injection-load" sweep only) or 50 (both buffer-size and "injection-load" 
sweeps). This is under the assumption that the bests-list is requested
to retain only one Attempt; this can be changed via the appropriate 
tool arguments (but the CPU runtime will increase accordingly).
During these simulations the GUI can be used to visualize the routers 
usage in real-time. If the GUI is utilized, then pictures of floorplans
are recorded automatically in the results/ directory as ".ps" files.
Note that the use of the GUI slows down the whole run.
After all the "Attemps" from the bests-list are simulated, a report is 
written to "results\results.txt". Reports in this file will start with 
the "name" of the experiment followed by a copy of the command line which
launced the tool. Then, results are recorded in a format similar to 
for example:

"BUFFERS SIZE: 1x

INJECTION LOAD: 60%
Attempt: 2	Flit delay: 2244.20
Attempt: 3	Flit delay: 2244.20
Attempt: 7	Flit delay: 2213.22

Average flit delay: 2233.87
Standard deviation: 14.61

Minimum flit delay: 2213.22"

Because a lot of results can be generated in this format, there is also 
an option to output summarized-data into an Excel file. This will record 
only the minimum latencies (or "flit delays").


General command to run the tool is as follows:
----------------------------------------------

./sfra file: (test file) cycles: (int) warmup: (int) [Options...]

These are the 3 mandatory options. The option "file:" should be given 
the name of the .fp testcase. Note that if an option is given multiple 
times, the last one is the interpreted one.


Examples:
---------

./sfra name: apte_simple1 file: tests/apte cycles: 60000 warmup: 1000 n_fps: 10 n_best: 3 mode: 2.5D scale: 1.366 times: 60 local: 10 avg_ratio: 20 load: 50 mode: 2.5D gui p: 1
./sfra name: apte_simple2 file: tests/apte cycles: 60000 warmup: 1000 n_fps: 10 n_best: 3 mode: 2.5D scale: 1.366 times: 60 local: 10 avg_ratio: 20 load: 50 mode: 2.5D gui p: 1 verbose: 0
./sfra name: apte_x_ary=2 file: tests/apte cycles: 60000 warmup: 1000 n_fps: 10 n_best: 3 mode: 2.5D scale: 1.366 times: 60 local: 10 avg_ratio: 20 load: 50 mode: 2.5D gui p: 2 x_ary: 2 verbose: 0
./sfra name: ami25_3D file: tests/ami25 cycles: 60000 warmup: 1000 n_fps: 10 n_best: 3 alpha: 0.25 fp_criteria: A scale: 5.512 seed: 1 mode: 3D times: 400 local: 7 avg_ratio: 40 load: 60 verbose: 0


For more examples on how to run it on various testcases, see the
".run" files under scripts/

To investigate the arcgitecture with 2 layers use "mode: 2.5D" and
to investigate the architecture with 3 layers use "mode: 3D".


Credits
=======
-- Vitor de Paulo (NDSU): helped me to integrate the VNOC simulator with
the B*tree floorplanner and to write parts of this README file;
-- Li Shang (while at Princeton) developed "popnet" from which I developed VNOC;
-- Jer-Ming Hsu, Hsun-Cheng Lee, and Yao-Wen Chang (of NCTU, Taiwan)
developed the "here-modified" B*-trees floorplanning tool;
-- Vaughn Betz (while at Univ. of Toronto) developed much of the GUI;
-- George Karypis (of Univ. of Minnesota) developed the hMetis partitioner;
-- Knuth's Stanford Graphbase: Hungarian Algorithm;


Copyright
=========
Copyright 2009 by Cristinel Ababei, cristinel.ababei@ndsu.edu
This Copyright notice applies to all files, called hereafter 
"The Software".
Permission to use, copy, and modify this software and its 
documentation is hereby granted only under the following 
terms and conditions.  Both the above copyright notice and 
this permission notice must appear in all copies of the 
software, derivative works or modified versions, and any 
portions thereof, and both notices must appear in supporting 
documentation.  Permission is granted only for non-commercial 
use.  For commercial use, please contact the author.
This software may be distributed (but not offered for sale 
or transferred for compensation) to third parties, provided 
such third parties agree to abide by the terms and conditions
of this notice.
The Software is provided "as is", and the authors, the 
North Dakota State University (NDSU), as well as any and
all previous authors (of portions or modified portions of
the software) disclaim all warranties with regard to this 
software, including all implied warranties of merchantability
and fitness.  In no event shall the authors or NDSU or any and
all previous authors be liable for any special, direct, 
indirect, or consequential damages or any damages whatsoever
resulting from loss of use, data or profits, whether in an
action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance
of this software.

