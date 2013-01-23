I. Dependency:

max-mixture only depends on g2o. You should be fine if g2o compiles. 
the version of g2o we support is the current master branch at github/g2o
(not the svn version).

===============================================================================

II. format for datasets:

The vertices remain unchanged 

EDGE_TYPE_MIXTURE va vb n EDGE_TAG w_1 edge_1 EDGE_TAG w_2 edge_2 ..
TYPE = {SE2,SE3:QUAT,POINTXY}
va, vb = vertices the edge currently connects. This will be constantly
            updated during each iteration.
n      = number of components this edge represents. If used only for the outlier
            rejection case n=2             
w_i    = weight used for ith edge
edge_i = standard g2o edge data of type = TYPE

===============================================================================
III. Compiling:

we recommend the "out of core" build.

cd max-mixture
mkdir build 
cmake ..
make

NOTE: set environment variable "G2O_ROOT" in case find_package(G2O) fails


===============================================================================


IV. Unimodal vs multi-modal:

Max-Mixture by formulation allows handling multi-modal edges.
It handles this by finding the most feasible mode/edge at every
iteration. Though simple to implement this changes the memory allocation
structure of the Jacobian at every iteration. 

1. Hence the solver.intialize() method must be called if multi-modal
formulation is used. Basically the user cannot call solve for more than
a single iteration since the underlying memory structure would change.
The compute error method for each mixture edge must be called (which updates the belief) before
building the structure. 

2. If using max-mixture only for outlier rejection the memory allocation
pattern does not change hence nothing needs to be done.

===============================================================================

V. Datasets:

Few example datasets are available inside max-mixtures/datasets


===============================================================================

VI Example:

~/g2o_viewer -typeslib path-to-max-mixture/lib/libg2o_max_mix_core.so dataset.g2o


===============================================================================

VII Known issue:

"Cholesky Decomposition is invalid". When using the landmark edges the
cholesky solver may return invalid results. Though not thoroughly
examined a plausible explanation is the following: If all the edges for
a particular landmark are rejected (uses the scaled down information
                                    matrix) the final weight
for the landmark nodes is very very small and the hessian graph may
become "numerically" disconnected. This can also happen for non-landmark
pose graph cases if all edges (loop+odometry) are max-mixture edges.

===============================================================================

VIII Bibtex:

@INPROCEEDINGS{Olson-RSS-12, 
    AUTHOR    = {Edwin Olson AND Pratik Agarwal}, 
    TITLE     = {Inference on networks of mixtures for robust robot mapping}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2012}, 
    ADDRESS   = {Sydney, Australia}, 
    MONTH     = {July} 
}

@misc{agarwal-MM,
    author = "Pratik Agarwal and Edwin Olson and Wolfram Burgard",
    title = "Max-mixture - open source implementation with g2o",
    howpublished = "\url{https://github.com/agpratik/max-mixture}",
    year = {2012},
}

===============================================================================

