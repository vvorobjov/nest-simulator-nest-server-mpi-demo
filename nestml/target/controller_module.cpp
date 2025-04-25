
/*
*  controller_module.cpp
*
*  This file is part of NEST.
*
*  Copyright (C) 2004 The NEST Initiative
*
*  NEST is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*
*  NEST is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with NEST.  If not, see <http://www.gnu.org/licenses/>.
*
*  2025-04-25 10:07:20.843938
*/

// Include from NEST
#include "nest_extension_interface.h"

// include headers with your own stuff


#include "tracking_neuron_nestml.h"

#include "basic_neuron_nestml.h"

#include "diff_neuron_nestml.h"



class controller_module : public nest::NESTExtensionInterface
{
  public:
    controller_module() {}
    ~controller_module() {}

    void initialize() override;
};

controller_module controller_module_LTX_module;

void controller_module::initialize()
{
    // register neurons
    register_tracking_neuron_nestml("tracking_neuron_nestml");
    register_basic_neuron_nestml("basic_neuron_nestml");
    register_diff_neuron_nestml("diff_neuron_nestml");
}
