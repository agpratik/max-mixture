
/*
 * Pratik Agarwal -- Max Mixture, the key thing here is that the graph can connect
 * multiple pair of nodes with only the required number of pairs being used in the optimization
 * The implementation becomes a little complicated when we try to model multiple data association
 * using a single edge
 */

#ifndef EDGE_SE3_MIXTURE
#define EDGE_SE3_MIXTURE

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/config.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

class EdgeSE3Mixture : public g2o::EdgeSE3
{
  public:

    EdgeSE3Mixture();
    EdgeSE3Mixture(std::vector< g2o::EdgeSE3* >& _edges, std::vector< double >& _weights);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os)const;
    virtual ~EdgeSE3Mixture();

    //void initializeComponents(std::vector<EdgeSE3Container*> data);
    void initializeComponents(std::vector<g2o::EdgeSE3*>& edges, std::vector<double>& weights);
    void UpdateBelief(int i);
    void computeError();
    //virtual void chi2();
    void linearizeOplus();
    
    void computeBestEdge();
    int getBestComponent()const;

    int numberComponents;
    EdgeSE3* getComponent(int i);
  private:

    double getNegLogProb(unsigned int c);

    //kind of a hack to read the vertices into allEdges once the mixture edge is added to the graph
    //could be solved if we have a pointer to the graph within the read function
    void updateVertexPairs();

    //out of the components which is the best one (max-probability)

    int bestComponent;
    std::vector<g2o::EdgeSE3*> allEdges;
    std::vector<double> weights;
    std::vector<double> determinants;
    bool verticesChanged;
    //to read vertices for subedges
    std::vector<std::pair<int,int> > vertexPairs;

    //bool onlyCorruptedGaussian = true; //if only corruptedGaussian then we dont need to allocate hessian memory
};


#ifdef G2O_HAVE_OPENGL
  class EdgeSE3MixtureDrawAction: public g2o::DrawAction{
  public:
    EdgeSE3MixtureDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
	  HyperGraphElementAction::Parameters* params_);
  };
#endif
#endif
