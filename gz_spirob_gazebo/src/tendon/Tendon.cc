/*
 * Copyright (C) 2026 Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Tendon.hh"

#include <gz/msgs/double.pb.h>

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::TendonPrivate
{
  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;
 
  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
Tendon::Tendon()
  : dataPtr(std::make_unique<TendonPrivate>())
{
}

//////////////////////////////////////////////////
void Tendon::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Tendon plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void Tendon::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Tendon::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;
}

GZ_ADD_PLUGIN(Tendon,
              System,
              Tendon::ISystemConfigure,
              Tendon::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Tendon,
                    "gz::sim::systems::Tendon")
