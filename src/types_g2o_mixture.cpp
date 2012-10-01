#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "types_g2o_mixture.h"

#include "edge_se2_mixture.h"
#include "edge_se2_pointxy_mixture.h"
#include "edge_se3_mixture.h"

using namespace g2o;

void init_types_mixture(void){
    Factory* factory = Factory::instance();
    factory->registerType("EDGE_SE2_MIXTURE", new HyperGraphElementCreator<EdgeSE2Mixture>);
    factory->registerType("EDGE_SE2_XY_MIXTURE", new HyperGraphElementCreator<EdgeSE2PointXYMixture>);
    factory->registerType("EDGE_SE3:QUAT_MIXTURE", new HyperGraphElementCreator<EdgeSE3Mixture>);
}

G2O_REGISTER_TYPE(EDGE_SE2_MIXTURE, EdgeSE2Mixture);
G2O_REGISTER_TYPE(EDGE_SE2_XY_MIXTURE, EdgeSE2PointXYMixture);
G2O_REGISTER_TYPE(EDGE_SE3:QUAT_MIXTURE, EdgeSE3Mixture);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(EdgeSE2MixtureDrawAction);
G2O_REGISTER_ACTION(EdgeSE2PointXYMixtureDrawAction);
G2O_REGISTER_ACTION(EdgeSE3MixtureDrawAction);
#endif

