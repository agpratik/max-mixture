#include <limits.h>
#include <math.h>
#include <GL/gl.h>

#include <g2o/core/factory.h>

#include "edge_se2_mixture.h"
#include "types_g2o_mixture.h"

EdgeSE2Mixture::EdgeSE2Mixture() : g2o::EdgeSE2::EdgeSE2()
{
  numberComponents = 0;
  verticesChanged = false;
  int bestComponent = -1;
}

EdgeSE2Mixture::~EdgeSE2Mixture()
{

}

EdgeSE2Mixture::EdgeSE2Mixture(std::vector< g2o::EdgeSE2* >& _edges, std::vector< double >& _weights) :g2o::EdgeSE2::EdgeSE2()
{
  initializeComponents(_edges,_weights);
}

void EdgeSE2Mixture::initializeComponents(std::vector< g2o::EdgeSE2* >& _edges, std::vector< double >& _weights)
{
  this->allEdges = _edges;
  this->weights = _weights;
  UpdateBelief(0);
  numberComponents = _edges.size();
   for(unsigned int i=0;i<numberComponents;i++)
     determinants.push_back(allEdges[i]->information().inverse().determinant());    
}

void EdgeSE2Mixture::UpdateBelief(int i)
{
  //required for multimodal max-mixtures
  if(allEdges[i]->vertex(0)!=_vertices[0] || allEdges[i]->vertex(1)!=_vertices[1]){
    //if(!_vertices[0] && !_vertices[1]) cerr << "\nVertices changed from |"<<_vertices[0]->id()<<" "<<_vertices[1]->id()<<"| ";    
    this->setVertex(0,allEdges[i]->vertex(0));
    this->setVertex(1,allEdges[i]->vertex(1));    
    //cerr <<"To |"<<_vertices[0]->id()<<" "<<_vertices[1]->id()<<"| ";
    verticesChanged = true;
    
    //constructQuadraticForm();
  }
  
  double p[3];
  allEdges[i]->getMeasurementData(p);
  this->setMeasurement(g2o::SE2(p[0],p[1],p[2]));
  this->setInformation(allEdges[i]->information());  
}

//get the edge with max probability and then create initialize the hessian 
//memory if the nodes have changed
void EdgeSE2Mixture::computeError()
{ 
  //xxx bad hack! can be changed with a small modofication in g2o
  updateVertexPairs();
  int best = -1;
  double minError = numeric_limits<double>::max();
  for(unsigned int i=0;i<numberComponents;i++){
    //allEdges[i]->computeError();
    double thisNegLogProb = getNegLogProb(i);
    if(minError>thisNegLogProb){
      best = i;
      minError = thisNegLogProb;
    }
  } 
  
  if(best!=bestComponent){
    bestComponent = best;
    UpdateBelief(bestComponent);
  }
  //_error = minError;
  g2o::EdgeSE2::computeError();
}

//very bad hack! can be changed with a slight modification of reading edges in g2o
void EdgeSE2Mixture::updateVertexPairs()
{
  for(unsigned int i=0;i<numberComponents;++i){
    VertexSE2* first = static_cast<VertexSE2*>(allEdges[i]->vertex(0));
    if(!first){
      allEdges[i]->setVertex(0,this->graph()->vertex(vertexPairs[i].first));
      allEdges[i]->setVertex(1,this->graph()->vertex(vertexPairs[i].second));
    }else{
      break;
    }
  }  
}

double EdgeSE2Mixture::getNegLogProb(unsigned int c)
{  
  allEdges[c]->computeError(); 
  //cerr << "\nchi2 for " <<c<<" "<< allEdges[c]->chi2();
  return -log(weights[c]) + 
	  0.5*log(determinants[c]) +
	  0.5*allEdges[c]->chi2();
}

void EdgeSE2Mixture::linearizeOplus()
{
  EdgeSE2Mixture::computeError();
  //the best component must be updated here 
  //else computeError can be called
  
  //UpdateBelief(bestComponent); //already done in compute error
  g2o::EdgeSE2::linearizeOplus();    
  //needs to be done since the vertex pair might have changed
  //Not sure if this is required 
  //it might be that we get this for free 
  //if(verticesChanged)
  //  g2o::EdgeSE2::constructQuadraticForm();  
}

int EdgeSE2Mixture::getBestComponent() const
{
  return bestComponent;
}

void EdgeSE2Mixture::computeBestEdge()
{
  computeError();
}

//note g2o takes care of creating the first two vertices 
//its problematic since all edges are taken care in this way 
//EDGE_SE2_MIXTURE na nb numComponents Edgetype_i w_i na_i nb_i 
bool EdgeSE2Mixture::read(std::istream& is)
{

  is >> numberComponents;
  
  allEdges.reserve(numberComponents);  
  weights.reserve(numberComponents);  
  determinants.reserve(numberComponents);
  vertexPairs.reserve(numberComponents);
  
  Vector3d p;
  double w;
  
  for(int c=0;c<numberComponents;c++){
    EdgeSE2* e = new EdgeSE2;    
    allEdges.push_back(e);
    std::string buf;
    is >> buf;    
    is>> w;        
    weights.push_back(w);
    int na,nb;
    is >> na;
    is >> nb;
    vertexPairs.push_back(std::pair<int,int>(na,nb));                
    //is>> weights[c];      
    is >> p[0] >> p[1] >> p[2];
    //cerr<<"\nMeasurement "<< p[0] <<" " <<p[1]<<" "<< p[2] <<" ";    
    allEdges[c]->setMeasurement(g2o::SE2(p));
    InformationType inf;
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
	is >> inf(i, j);
	if (i != j)
	  inf(j, i) = inf(i, j);
      }
      allEdges[c]->setInformation(inf);      
  }
    
  for(unsigned int i=0;i<numberComponents;i++)
     determinants.push_back(allEdges[i]->information().inverse().determinant());    
  UpdateBelief(0);
  return is.good();
}


bool EdgeSE2Mixture::write(std::ostream& os) const
{
  os << numberComponents << " ";
  g2o::Factory* factory = g2o::Factory::instance();
  
  double p[3];
  //cerr <<"\n";
  for(int c=0;c<numberComponents;c++){
    os<<factory->tag(allEdges[c])<<" ";
    os<< weights[c]<<" ";
    //cerr<< "Writing component "<< c << " ";
    os<<allEdges[c]->vertex(0)->id()<<" "; 
    os<<allEdges[c]->vertex(1)->id()<<" ";
    //os >> this->_vertices[0]->id()<<" ";
    //os << this->_vertices[1]->id()<< " ";
    
    allEdges[c]->getMeasurementData(p);
    os<<p[0] <<" " <<p[1]<<" "<< p[2] <<" ";
    InformationType inf;    
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)	
	os << " " << allEdges[c]->information()(i, j);
      
      os<<" ";
  }
  return os.good();
}

EdgeSE2* EdgeSE2Mixture::getComponent(int i)
{
  return allEdges[i];
}


  #ifdef G2O_HAVE_OPENGL
  EdgeSE2MixtureDrawAction::EdgeSE2MixtureDrawAction(): DrawAction(typeid(EdgeSE2Mixture).name()){}
  
   HyperGraphElementAction* EdgeSE2MixtureDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

//     EdgeSE2Mixture* e =  static_cast<EdgeSE2Mixture*>(element);
//     VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
//     VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
//     //glColor3f(0.5f,0.5f,0.8f);
//     if(e->numberComponents==2){
//     if(e->getBestComponent()==0)//accepted 
//       glColor3f(0.5f,0.0f,0.0f);
//     else//rejected 
//       glColor3f(1.0f,0.8f,0.8f);
//     }
//     glPushAttrib(GL_ENABLE_BIT);
//     glDisable(GL_LIGHTING);
//     glBegin(GL_LINES);
//     glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
//     glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),0.f);
//     glEnd();
//     glPopAttrib();
//     return this;
    
        glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    
    EdgeSE2Mixture* e =  static_cast<EdgeSE2Mixture*>(element);


    glBegin(GL_LINES);
    if(e->numberComponents>2){
      //BEST EDGE
      VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
      VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
      glColor3f(0.8f,0.0f,1.0f);
      glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
      glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),0.f);            
      
//       for(unsigned int i=0;i<e->numberComponents;++i){
// 	if(i==e->getBestComponent())
// 	  continue;
// 	VertexSE2* fromEdge = static_cast<VertexSE2*>(e->getComponent(i)->vertex(0));
// 	VertexSE2* toEdge   = static_cast<VertexSE2*>(e->getComponent(i)->vertex(1));
// 	if(fromEdge && toEdge){
// 	  glColor3f(1.0f,0.8f,1.0f);//rejected edges
// 	  glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
// 	  glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),0.f);            
// 	}
//       }
     }
    else{
      if(e->getBestComponent()==0)
	glColor3f(0.8f,0.0f,1.0f);
      else
	glColor3f(1.0f,0.8f,1.0f);
      
      VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
      VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertex(1));
      
      glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
      glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),0.f);            
    }
    
    glEnd();
    glPopAttrib();
    return this;
    
  }
  #endif


