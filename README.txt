I. Dependency:

max-mixture only depends on g2o. You should be fine if g2o compiles. 
the version of g2o we support is the current master branch at github/g2o
(not the svn version)

===============================================================================

II. format for datasets:

The vertices remain unchanged 

EDGE_TYPE_MIXTURE va vb n w_1 edge_1 w_2 edge_2 ..
TYPE = {SE2,SE3:QUAT,POINTXY}
va, vb = vertices the edge currently connects. This will be constantly
            updated during each iteration.
n      = number of components this edge represents. If used only for the outlier
            rejection case n=2             
w_i    = weight used for ith edge
edge_i = standard g2o edge data of type = TYPE

===============================================================================
III. Building:

we recommend the "out of core" build.

cd max-mixture
mkdir build 
cmake ..
make

NOTE: set environment variable "G2O_ROOT" in case find_package(G2O) fails


===============================================================================


IV. Unimodal vs multi-modal:

Max-Mixture by formulation allows handling multi-modal edges.
It handles this by find the most feasible mode/edge at every
iteration. Though simple to implement this changes the memory allocation
structure of the Jacobian at every iteration.

1. Hence the intialize() method must be called if multi-modal
formulation is used. Basically the user cannot call solve for more than
a single step. You need to call solve with one iteration within a loop.

2. If using max-mixture only for outlier rejection the memory allocation
pattern does not change -- hence initialize() need not be called. 

===============================================================================

V. Datasets:
Few example datasets are available inside max-mixtures/datasets


===============================================================================

VI Example:

~/g2o_viewer -typeslib


===============================================================================
