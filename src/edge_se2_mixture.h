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

/*
 * Pratik Agarwal -- Max Mixture, the key thing here is that the graph can connect 
 * multiple pair of nodes with only the required number of pairs being used in the optimization
 * The implementation becomes a little complicated when we try to model multiple data association 
 * using a single edge
 */

#ifndef EDGE_SE2_MIXTURE
#define EDGE_SE2_MIXTURE

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/config.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

class EdgeSE2Mixture : public g2o::EdgeSE2
{
  public:
        
    EdgeSE2Mixture();
    EdgeSE2Mixture(std::vector< g2o::EdgeSE2* >& _edges, std::vector< double >& _weights);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os)const;
    virtual ~EdgeSE2Mixture();
    
    //void initializeComponents(std::vector<EdgeSE2Container*> data);
    void initializeComponents(std::vector<g2o::EdgeSE2*>& edges, std::vector<double>& weights);
    void UpdateBelief(int i);
    virtual void computeError();
    virtual void linearizeOplus();
    
    void computeBestEdge();
    int getBestComponent()const;
    EdgeSE2* getComponent(int i);
    
    int numberComponents;
    
  private:

    double getNegLogProb(unsigned int c);
    
    //kind of a hack to read the vertices into allEdges once the mixture edge is added to the graph
    //could be solved if we have a pointer to the graph within the read function 
    void updateVertexPairs();
    
    //out of the components which is the best one (max-probability)    
    
    int bestComponent;    
    std::vector<g2o::EdgeSE2*> allEdges;
    std::vector<double> weights; 
    std::vector<double> determinants;
    bool verticesChanged;
    //to read vertices for subedges
    std::vector<std::pair<int,int> > vertexPairs;
    
    //bool onlyCorruptedGaussian = true; //if only corruptedGaussian then we dont need to allocate hessian memory
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