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

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "types_g2o_mixture.h"

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

