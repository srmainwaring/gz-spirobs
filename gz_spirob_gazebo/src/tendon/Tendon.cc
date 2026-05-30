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
#include <thread>
#include <vector>

#include <gz/common/Profiler.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Link.hh"
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;


// Struture to store information about a link segment
struct TendonSegment
{
  /// \brief The first link to which the tendon attaches.
  public: Link link1;
  /// \brief The second link to which the tendon attaches.
  public: Link link2;
  /// \brief Name of the first link.
  public: std::string link1Name;
  /// \brief Name of the second link.
  public: std::string link2Name;
  /// \brief The first link tendon attachment site in the link frame.
  public: math::Vector3d site1;
  /// \brief The second link tendon attachment site in the link frame.
  public: math::Vector3d site2;
};


class gz::sim::systems::TendonPrivate
{
  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Callback for tendon force subscription
  /// \param[in] _msg Tendon force message
  public: void OnCmdForce(const msgs::Double &_msg);

  /// \brief Tendon name
  // @todo remove hardcode
  public: std::string tendonName;

  /// \brief Commanded force
  public: double tendonForceCmd;

  /// \brief mutex to protect tendonForceCmd
  public: std::mutex tendonForceCmdMutex;

  /// \brief A list of the segments comprising the tendon.
  public: std::vector<TendonSegment> tendonSegments;
  /// \brief Set to true when all tendon segments are initialised.
  public: bool isValid = false;

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

  // Read plugin SDF
  auto sdfClone = _sdf->Clone();

  // Get params from SDF
  auto sdfTendon = sdfClone->GetElement("tendon");
  if (!sdfTendon)
  {
    gzerr << "Tendon plugin must contain a <tendon> element." << std::endl;
    return;
  }

  auto sdfTendonName = sdfTendon->GetElement("name");
  if (!sdfTendonName)
  {
    gzerr << "Tendon must contain a <name> element." << std::endl;
    return;
  }

  if (!sdfTendonName->Get<std::string>().empty())
  {
    this->dataPtr->tendonName = sdfTendonName->Get<std::string>();
  }
  else
  {
    gzerr << "<name> provided but is empty." << std::endl;
    return;
  }

  auto sdfSegment = sdfTendon->GetElement("segment");
  while (sdfSegment)
  {
    // Check segment contains required elements
    auto sdfLink1 = sdfSegment->GetElement("link1");
    if (!sdfLink1)
    {
      gzerr << "Tendon segment must contain a <link1> element." << std::endl;
      return;
    }
    auto sdfLink2 = sdfSegment->GetElement("link2");
    if (!sdfLink2)
    {
      gzerr << "Tendon segment must contain a <link2> element." << std::endl;
      return;
    }
    auto sdfSite1 = sdfSegment->GetElement("site1");
    if (!sdfSite1)
    {
      gzerr << "Tendon segment must contain a <site1> element." << std::endl;
      return;
    }
    auto sdfSite2 = sdfSegment->GetElement("site2");
    if (!sdfSite2)
    {
      gzerr << "Tendon segment must contain a <site2> element." << std::endl;
      return;
    }

    TendonSegment segment;

    if (!sdfLink1->Get<std::string>().empty())
    {
      segment.link1Name = sdfLink1->Get<std::string>();
    }
    else
    {
      gzerr << "<link1> provided but is empty." << std::endl;
      return;
    }

    if (!sdfLink2->Get<std::string>().empty())
    {
      segment.link2Name = sdfLink2->Get<std::string>();
    }
    else
    {
      gzerr << "<link2> provided but is empty." << std::endl;
      return;
    }

    segment.site1 = sdfSite1->Get<math::Vector3d>();
    segment.site2 = sdfSite2->Get<math::Vector3d>();
    this->dataPtr->tendonSegments.push_back(segment);

    sdfSegment = sdfSegment->GetNextElement("segment");

    // Debug
    gzdbg << "Adding segment" << std::endl;
    gzdbg << "link1Name:      " << segment.link1Name << std::endl;
    gzdbg << "link2Name:      " << segment.link2Name << std::endl;
    gzdbg << "site1:          " << segment.site1 << std::endl;
    gzdbg << "site2:          " << segment.site2 << std::endl << std::endl;
  }

  // Subscribe to commands
  auto topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/tendon/" + this->dataPtr->tendonName +
      "/cmd_force");
  if (topic.empty())
  {
    gzerr << "Failed to create valid topic for [" << this->dataPtr->tendonName
           << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic, &TendonPrivate::OnCmdForce,
                                this->dataPtr.get());

  gzmsg << "Tendon subscribing to Double messages on [" << topic
         << "]" << std::endl;

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

  // If the links have not been identified, look for them
  if (!this->dataPtr->isValid)
  {
    for (auto && segment: this->dataPtr->tendonSegments)
    {
      this->dataPtr->isValid = true;
      if (!segment.link1.Valid(_ecm))
      {
        segment.link1 =
            Link(this->dataPtr->model.LinkByName(_ecm, segment.link1Name));
      }
      if (!segment.link1.Valid(_ecm))
      {
        this->dataPtr->isValid = false;
        return;
      }

      if (!segment.link2.Valid(_ecm))
      {
        segment.link2 =
            Link(this->dataPtr->model.LinkByName(_ecm, segment.link2Name));
      }
      if (!segment.link2.Valid(_ecm))
      {
        this->dataPtr->isValid = false;
        return;
      }
    }
  }

  int i = 0;
  for (auto && segment: this->dataPtr->tendonSegments)
  {
    // Calculate the site position in world coordinates
    math::Pose3d worldPoseLink1 = segment.link1.WorldPose(_ecm).value();
    math::Pose3d worldPoseLink2 = segment.link2.WorldPose(_ecm).value();

    // Calculate world pose of site1 and site2
    math::Pose3d bodyPoseSite1 = math::Pose3d(
        segment.site1, math::Quaterniond::Identity);
    math::Pose3d bodyPoseSite2 = math::Pose3d(
        segment.site2, math::Quaterniond::Identity);
    math::Pose3d worldPoseSite1 = worldPoseLink1 * bodyPoseSite1;
    math::Pose3d worldPoseSite2 = worldPoseLink2 * bodyPoseSite2;

    // Calculate the force along the tendon directed from site1 to site2
    math::Vector3d r12 = worldPoseSite2.Pos() - worldPoseSite1.Pos();
    math::Vector3d u12 = r12.Normalize();
    math::Vector3d f12 = this->dataPtr->tendonForceCmd * u12;
    math::Vector3d f21 = -1.0 * f12;

    // Apply forces at site points in world frame
    segment.link1.AddWorldWrench(_ecm, f12, math::Vector3d::Zero,
        segment.site1);
    segment.link2.AddWorldWrench(_ecm, f21, math::Vector3d::Zero,
        segment.site2);

    // Debug - the last n segments
    //int n = 1;
    //if (i >= this->dataPtr->tendonSegments.size() - n)
    //{
    //  gzdbg << "Link1:          " << segment.link1.Name(_ecm).value()
    //                              << std::endl;
    //  gzdbg << "Link2:          " << segment.link2.Name(_ecm).value()
    //                              << std::endl;
    //  gzdbg << "worldPoseLink1: " << worldPoseLink1 << std::endl;
    //  gzdbg << "worldPoseLink2: " << worldPoseLink2 << std::endl;
    //  gzdbg << "worldPoseSite1: " << worldPoseSite1 << std::endl;
    //  gzdbg << "worldPoseSite2: " << worldPoseSite2 << std::endl;
    //  gzdbg << "u12:              " << u12 << std::endl;
    //  gzdbg << "f12:              " << f12 << std::endl;
    //  gzdbg << "f21:              " << f21 << std::endl << std::endl;
    //}
    i++;
  }
}

//////////////////////////////////////////////////
void TendonPrivate::OnCmdForce(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->tendonForceCmdMutex);
  this->tendonForceCmd = _msg.data();
}

GZ_ADD_PLUGIN(Tendon,
              System,
              Tendon::ISystemConfigure,
              Tendon::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Tendon,
                    "gz::sim::systems::Tendon")
