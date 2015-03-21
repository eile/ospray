// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "sg/common/TransferFunction.h"

namespace ospray {
  namespace sg {

    /*! a geometry node - the generic geometry node */
    struct Volume : public sg::Node {
      Volume() {};

      /*! \brief returns a std::string with the c++ name of this class */
      virtual    std::string toString() const { return "ospray::sg::Volume"; }

      //! return bounding box of all primitives
      virtual box3f getBounds() = 0;

      SG_NODE_DECLARE_MEMBER(Ref<TransferFunction>,transferFunctionfovy,TransferFunction);    
    };
    
  } // ::ospray::sg
} // ::ospray

