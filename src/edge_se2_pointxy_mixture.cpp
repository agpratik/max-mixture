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
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

#include <limits.h>
#include <math.h>
#include <GL/gl.h>

#include <g2o/core/factory.h>

#include "edge_se2_pointxy_mixture.h"
#include "types_g2o_mixture.h"

EdgeSE2PointXYMixture::EdgeSE2PointXYMixture() : g2o::EdgeSE2PointXY::EdgeSE2PointXY()
{
  numberComponents = 0;
  int bestComponent = -1;
}

EdgeSE2PointXYMixture::~EdgeSE2PointXYMixture()
{

}

EdgeSE2PointXYMixture::EdgeSE2PointXYMixture(std::vector< g2o::EdgeSE2PointXY* >& _edges, std::vector< double >& _weights) :g2o::EdgeSE2PointXY::EdgeSE2PointXY()
{
  initializeComponents(_edges,_weights);
}

void EdgeSE2PointXYMixture::initializeComponents(std::vector< g2o::EdgeSE2PointXY* >& _edges, std::vector< double >& _weights)
{
  this->allEdges = _edges;
  this->weights = _weights;
  //UpdateBelief(0);
  numberComponents = _edges.size();
   for(unsigned int i=0;i<numberComponents;i++)
     determinants.push_back(allEdges[i]->information().inverse().determinant());

   computeBestEdge();
}

void EdgeSE2PointXYMixture::UpdateBelief(int i)
{
  //required for multimodal max-mixtures, this part may be optimized for speed,
  //do this only when component changed.
  this->setVertex(0,allEdges[i]->vertex(0));
  this->setVertex(1,allEdges[i]->vertex(1));

  double p[2];
  allEdges[i]->getMeasurementData(p);
  this->setMeasurementData(p);
  this->setInformation(allEdges[i]->information());
}

//get the edge with max probability and then create initialize the hessian
//memory if the nodes have changed
void EdgeSE2PointXYMixture::computeError()
{
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

  //if(best!=bestComponent){
    bestComponent = best;
    UpdateBelief(bestComponent);
  //}
  g2o::EdgeSE2PointXY::computeError();
}

double EdgeSE2PointXYMixture::getNegLogProb(unsigned int c)
{
  allEdges[c]->computeError();
  //cerr << "\nchi2 for " <<c<<" "<< allEdges[c]->chi2();
  return -log(weights[c]) +
	  0.5*log(determinants[c]) +
	  0.5*allEdges[c]->chi2();
}

void EdgeSE2PointXYMixture::linearizeOplus()
{
  EdgeSE2PointXY::computeError();
  g2o::EdgeSE2PointXY::linearizeOplus();
}

int EdgeSE2PointXYMixture::getBestComponent() const
{
  return bestComponent;
}

void EdgeSE2PointXYMixture::computeBestEdge()
{
  computeError();
}

//note g2o takes care of creating the first two vertices
//its problematic since all edges are taken care in this way
//EDGE_SE2_MIXTURE na nb numComponents Edgetype_i w_i na_i nb_i
bool EdgeSE2PointXYMixture::read(std::istream& is)
{

  is >> numberComponents;

  allEdges.reserve(numberComponents);
  weights.reserve(numberComponents);
  determinants.reserve(numberComponents);

  double p[2];
  double w;

  //might throw error if first vertex is landmark
  VertexSE2* va = static_cast<VertexSE2*>(this->vertex(0));
  assert(va!=NULL);
  
  for(int c=0;c<numberComponents;c++){
    EdgeSE2PointXY* e = new EdgeSE2PointXY;
    allEdges.push_back(e);
    std::string buf;
    is >> buf;
    is>> w;
    weights.push_back(w);
    int na,nb;
    is >> na;
    is >> nb;

    VertexSE2* v0 = static_cast<VertexSE2*>(va->graph()->vertex(na));
    VertexPointXY* v1 = static_cast<VertexPointXY*>(va->graph()->vertex(nb));
    assert(v0!=NULL);
    assert(v1!=NULL);
    allEdges[c]->setVertex(0,v0);
    allEdges[c]->setVertex(1,v1);
    
    //is>> weights[c];
    is >> p[0] >> p[1];
    //cerr<<"\nMeasurement "<< p[0] <<" " <<p[1]<<" "<< p[2] <<" ";
    allEdges[c]->setMeasurementData(p);
    
    InformationType inf;
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j) {
	is >> inf(i, j);
	if (i != j)
	  inf(j, i) = inf(i, j);
      }
      allEdges[c]->setInformation(inf);
  }

  for(unsigned int i=0;i<numberComponents;i++)
     determinants.push_back(allEdges[i]->information().inverse().determinant());

  computeBestEdge();
  return is.good();
}

EdgeSE2PointXY* EdgeSE2PointXYMixture::getComponent(unsigned int i)
{
  return allEdges[i];
}



bool EdgeSE2PointXYMixture::write(std::ostream& os) const
{
  os << numberComponents << " ";
  g2o::Factory* factory = g2o::Factory::instance();

  double p[2];
  //cerr <<"\n";
  for(int c=0;c<numberComponents;c++){
    os<<factory->tag(allEdges[c])<<" ";
    os<< weights[c]<<" ";
    //cerr<< "\nWriting component "<< c << " ";
    os<<allEdges[c]->vertex(0)->id()<<" ";
    //cerr<< "\nid is "<<allEdges[c]->vertex(0)->id();
    os<<allEdges[c]->vertex(1)->id()<<" ";
    //cerr<< "id is "<<allEdges[c]->vertex(1)->id();
    //os >> this->_vertices[0]->id()<<" ";
    //os << this->_vertices[1]->id()<< " ";

    allEdges[c]->getMeasurementData(p);
    os<<p[0] <<" " <<p[1]<<" ";
    InformationType inf;
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j)
	os << " " << allEdges[c]->information()(i, j);

      os<<" ";
  }
  return os.good();
}

  #ifdef G2O_HAVE_OPENGL
  EdgeSE2PointXYMixtureDrawAction::EdgeSE2PointXYMixtureDrawAction(): DrawAction(typeid(EdgeSE2PointXYMixture).name()){}

   HyperGraphElementAction* EdgeSE2PointXYMixtureDrawAction::operator()(HyperGraph::HyperGraphElement* element,
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    
    EdgeSE2PointXYMixture* e =  static_cast<EdgeSE2PointXYMixture*>(element);

    glBegin(GL_LINES);
    
    if(e->numberComponents>2){
      //BEST EDGE
      VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
      VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
      glColor3f(0.8f,0.0f,1.0f);
      glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
      glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);           
      
      for(unsigned int i=0;i<e->numberComponents;++i){
	if(i==e->getBestComponent())
	  continue;
	VertexSE2* fromEdge = static_cast<VertexSE2*>(e->getComponent(i)->vertex(0));
	VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->getComponent(i)->vertex(1));
	if(fromEdge && toEdge){
	  glColor3f(1.0f,0.8f,1.0f);//rejected edges
	  glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
	  glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);            
	}
      }
    }else{//accepted rejected
      if(e->getBestComponent()==0)
	glColor3f(0.8f,0.0f,1.0f);
      else
	glColor3f(1.0f,0.8f,1.0f);
      VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
      VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
     
      glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
      glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),0.f);           
    }
      
     
    glEnd();
    glPopAttrib();
    return this;
  }
  #endif



