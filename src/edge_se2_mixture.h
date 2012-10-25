// Max-Mixture plugin for g2o
// Copyright (C) 2012 P. Agarwal, E. Olson, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef EDGE_SE2_MIXTURE
#define EDGE_SE2_MIXTURE

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/config.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

/**
 * SE2 Max-Mixture
 */
class EdgeSE2Mixture : public g2o::EdgeSE2
{
  public:
    /**
     * Deafult constructor
     */    
    EdgeSE2Mixture();
    
    /**
     * Constructor
     * @param _edges each se2edge with nodes and information already initialized
     * @param _weight by default we chose the weight of the first component to be 1
     */
    EdgeSE2Mixture(std::vector< g2o::EdgeSE2* >& _edges, std::vector< double >& _weights);
        
    bool read(std::istream& is);
    bool write(std::ostream& os)const;
    virtual ~EdgeSE2Mixture();
    
    
    void initializeComponents(std::vector<g2o::EdgeSE2*>& edges, std::vector<double>& weights);
    
    /**
     * Updates the current belief to the component numbered i
     * @param i component number
     */
    void UpdateBelief(int i);
        
    void computeError();
    void linearizeOplus();
    
    /**
     * Computes the best component
     */
    void computeBestEdge();
    
    /**
     * @return the best component 
     */
    int getBestComponent()const;
    
    /**
     * return the edge corresponding to component i
     * here to compute stats regarding the operation
     */
    EdgeSE2* getComponent(int i);
    
    int numberComponents;
    
  private:

    /**
     * Computes the neglative log likelihood of a given component
     * @param c component whole Negative log should be computed
     */
    double getNegLogProb(unsigned int c);        
    
    //out of the components which is the best one (max-probability)    
    
    int bestComponent;    
    std::vector<g2o::EdgeSE2*> allEdges;
    std::vector<double> weights; 
    std::vector<double> determinants;
    
    bool verticesChanged;
};


#ifdef G2O_HAVE_OPENGL
  class EdgeSE2MixtureDrawAction: public g2o::DrawAction{
  public:
    EdgeSE2MixtureDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
	  HyperGraphElementAction::Parameters* params_);
  };
#endif
#endif