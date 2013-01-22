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

#ifndef EDGE_SE2_POINTXY_MIXTURE
#define EDGE_SE2_POINTXY_MIXTURE

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/config.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

class EdgeSE2PointXYMixture : public g2o::EdgeSE2PointXY
{
  public:

    EdgeSE2PointXYMixture();
    EdgeSE2PointXYMixture(std::vector< g2o::EdgeSE2PointXY* >& _edges, std::vector< double >& _weights);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os)const;
    virtual ~EdgeSE2PointXYMixture();

    //void initializeComponents(std::vector<EdgeSE2PointXYContainer*> data);
    void initializeComponents(std::vector<g2o::EdgeSE2PointXY*>& edges, std::vector<double>& weights);
    void UpdateBelief(int i);
    void computeError();
    void linearizeOplus();

    void computeBestEdge();
    int getBestComponent()const;

    int numberComponents;

    EdgeSE2PointXY* getComponent(unsigned int i);
    
  private:

    double getNegLogProb(unsigned int c);

    //out of the components which is the best one (max-probability)

    int bestComponent;
    std::vector<g2o::EdgeSE2PointXY*> allEdges;
    std::vector<double> weights;
    std::vector<double> determinants;
    bool verticesChanged;
    //to read vertices for subedges
    std::vector<std::pair<int,int> > vertexPairs;

    //bool onlyCorruptedGaussian = true; //if only corruptedGaussian then we dont need to allocate hessian memory
};


#ifdef G2O_HAVE_OPENGL
  class EdgeSE2PointXYMixtureDrawAction: public g2o::DrawAction{
  public:
    EdgeSE2PointXYMixtureDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
	  HyperGraphElementAction::Parameters* params_);
  };
#endif
#endif
