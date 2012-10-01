
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