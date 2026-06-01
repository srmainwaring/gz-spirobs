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

namespace
{
  template <typename T>
  int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }
}

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

  /// \brief Track changes in tendon length at each site.
  public: std::vector<math::Vector3d> segmentDirs;
  //public: std::vector<double> segmentLengths;
  //public: std::vector<double> sumSegmentLengths;
  //public: std::vector<double> prevSegmentLengths;
  //public: std::vector<double> prevSumSegmentLengths;
  //public: std::vector<double> deltaSumSegmentLengths;

  // AI (ChatGPT)
  public: std::vector<double> T_in;
  public: std::vector<double> T_out;
  public: std::vector<double> T_prev_out;
  public: std::vector<bool> sliding;

  /// \brief Friction coefficient.
  public: double mu = 0.1;

  /// \brief Slip activation gain.
  public: double alpha_scale = 0.1;

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

  if (sdfTendon->HasElement("mu"))
  {
      double mu = sdfTendon->Get<double>("mu");
      if (mu >= 0.0)
      {
        this->dataPtr->mu = mu;
      }
  }

  if (sdfTendon->HasElement("alpha_scale"))
  {
      double alpha_scale = sdfTendon->Get<double>("alpha_scale");
      if (alpha_scale >= 0.0)
      {
        this->dataPtr->alpha_scale = alpha_scale;
      }
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

  // Resize segment workspace
  const auto segmentCount = this->dataPtr->tendonSegments.size();
  this->dataPtr->segmentDirs.resize(segmentCount);
  //this->dataPtr->segmentLengths.resize(segmentCount);
  //this->dataPtr->sumSegmentLengths.resize(segmentCount);
  //this->dataPtr->prevSegmentLengths.resize(segmentCount);
  //this->dataPtr->prevSumSegmentLengths.resize(segmentCount);
  //this->dataPtr->deltaSumSegmentLengths.resize(segmentCount);

  //AI
  this->dataPtr->T_in.resize(segmentCount, 0.0);
  this->dataPtr->T_out.resize(segmentCount, 0.0);
  this->dataPtr->T_prev_out.resize(segmentCount, 0.0);
  this->dataPtr->sliding.resize(segmentCount, false);

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
  bool isFirstPass = false;
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

    // All links identified, confirm first pass
    isFirstPass = true;
  }

  const auto segmentCount = this->dataPtr->tendonSegments.size();
  double sumSegmentLength = 0.0;
  for (auto i = 0; i < segmentCount; ++i)
  {
    auto && segment = this->dataPtr->tendonSegments[i];

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
    math::Vector3d u12 = r12.Normalized();
    math::Vector3d f12 = this->dataPtr->tendonForceCmd * u12;
    math::Vector3d f21 = -1.0 * f12;

    double segmentLength = r12.Length();
    sumSegmentLength += segmentLength;
    this->dataPtr->segmentDirs[i] = u12;
    //this->dataPtr->segmentLengths[i] = segmentLength;
    //this->dataPtr->sumSegmentLengths[i] = sumSegmentLength;

    // AI
    // @TODO add mutex
    this->dataPtr->T_in[0] = this->dataPtr->tendonForceCmd;
    this->dataPtr->T_out[0] = this->dataPtr->tendonForceCmd;

    //gzdbg << "r12: " << r12 << std::endl;

    // On first pass also set state for previous step 
    //if (isFirstPass)
    //{
    //  this->dataPtr->prevSegmentLengths[i] = segmentLength;
    //  this->dataPtr->prevSumSegmentLengths[i] = sumSegmentLength;
    //  this->dataPtr->deltaSumSegmentLengths[i] = 0.0;
    //}
  }

  // Now calculate the changes in the length to the fixed end point.
  //double firstSegmentLength = 0.0;
  //double prevFirstSegmentLength = 0.0;
  //double prevSumSegmentLength = 0.0;
  //if (segmentCount > 0.0)
  //{
  //  firstSegmentLength = this->dataPtr->segmentLengths[0];
  //  prevFirstSegmentLength = this->dataPtr->prevSegmentLengths[0];
  //  prevSumSegmentLength = this->dataPtr->prevSumSegmentLengths.back();
  //}
  //gzdbg << "L: " << sumSegmentLength << std::endl;
  //for (auto i = 0; i < segmentCount; ++i)
  //{
    // Distance from the start of of each segment to the fixed end point
    //double length = sumSegmentLength +
    //    firstSegmentLength - this->dataPtr->sumSegmentLengths[i];

    //double prevLength = prevSumSegmentLength +
    //    prevFirstSegmentLength - this->dataPtr->prevSumSegmentLengths[i];

    // dL indicates the direction the tendon is moving through the site at
    // the start of the segment.
    // dL > 0, the site is moving away from the fixed end (relaxing)
    // dL < 0, the site is moving towards the fixed end (contracting)
    //double dL = length - prevLength;
    //this->dataPtr->deltaSumSegmentLengths[i] = dL;

    // Cycle
    //this->dataPtr->prevSegmentLengths[i]
    //    = this->dataPtr->segmentLengths[i];
    //this->dataPtr->prevSumSegmentLengths[i]
    //    = this->dataPtr->sumSegmentLengths[i];
  //}
  //gzdbg << std::endl;

  // Apply forces
#if 0
  double cmdForce{0.0};
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->tendonForceCmdMutex);
    cmdForce = this->dataPtr->tendonForceCmd;
  }

  for (auto i = 0; i < segmentCount; ++i)
  {
    auto && segment = this->dataPtr->tendonSegments[i];

    // wrap angle
    double phi = 0.0;
    if (i > 0)
    {
      math::Vector3d d1 = this->dataPtr->segmentDirs[i - 1];
      math::Vector3d d2 = this->dataPtr->segmentDirs[i];
      phi = std::acos(d1.Dot(d2));
    }

    // direction of tendon movement through site gamma < 1 (contracting)
    double gamma = sgn(this->dataPtr->deltaSumSegmentLengths[i]);

    // apply capstan formula for scaling tension 
    double capstanFactor = std::exp(gamma * this->dataPtr->mu * phi);
    cmdForce *= capstanFactor;

    // Debug
    gzdbg << "L[" << i << "]: " << this->dataPtr->sumSegmentLengths[i]
          << ", dL: " << this->dataPtr->deltaSumSegmentLengths[i]
          << ", gamma: " << gamma
          << ", phi: " << (phi * 180.0 / M_PI)
          << ", capstan: " << capstanFactor
          << ", force: " << cmdForce
          << std::endl;

    math::Vector3d u12 = this->dataPtr->segmentDirs[i];
    math::Vector3d f12 = cmdForce * u12;
    math::Vector3d f21 = -1.0 * f12;

    // Apply forces at site points in world frame
    segment.link1.AddWorldWrench(_ecm, f12, math::Vector3d::Zero,
        segment.site1);
    segment.link2.AddWorldWrench(_ecm, f21, math::Vector3d::Zero,
        segment.site2);
  }
#endif
  // AI
  double T0_cmd;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->tendonForceCmdMutex);
    T0_cmd = this->dataPtr->tendonForceCmd;
  }

  // TODO must check segmentCount > 0 or short circult update.
  // Boundary condition (actuator side)
  this->dataPtr->T_in[0] = T0_cmd;

  for (auto i = 0; i < segmentCount; ++i)
  {
    auto &segment = this->dataPtr->tendonSegments[i];

    //// wrap angle
    double phi = 0.0;
    if (i > 0)
    {
      math::Vector3d d1 = this->dataPtr->segmentDirs[i - 1];
      math::Vector3d d2 = this->dataPtr->segmentDirs[i];
      phi = std::acos(std::clamp(d1.Dot(d2), -1.0, 1.0));
    }

    //double Tin = this->dataPtr->T_in[i];

    //// capstan bounds
    //double Tmin = Tin * std::exp(-this->dataPtr->mu * phi);
    //double Tmax = Tin * std::exp(+this->dataPtr->mu * phi);

    //double Told = this->dataPtr->T_out[i];

    //// predicted sliding update (direction from previous step)
    //double Tpred = Tin * std::exp(this->dataPtr->mu * phi *
    //                              (Told >= Tin ? 1.0 : -1.0));

    //// --- stick-slip decision ---
    //bool stick = (Told >= Tmin && Told <= Tmax);

    //double Tout;

    //if (stick)
    //{
    //  // STICK: preserve previous state
    //  Tout = Told;
    //  this->dataPtr->sliding[i] = false;
    //}
    //else
    //{
    //  // SLIP: project onto capstan boundary
    //  if (Told < Tmin)
    //    Tout = Tmin;
    //  else
    //    Tout = Tmax;

    //  this->dataPtr->sliding[i] = true;
    //}

    //this->dataPtr->T_out[i] = Tout;

    //// propagate to next segment
    //if (i + 1 < segmentCount)
    //{
    //  this->dataPtr->T_in[i + 1] = Tout;
    //}

    double Tin = this->dataPtr->T_in[i];
    double Told = this->dataPtr->T_out[i];

    double mu = this->dataPtr->mu;
    double Tmin = Tin * exp(-mu * phi);
    double Tmax = Tin * exp(+mu * phi);

    // state-based direction
    double sigma = (Told >= Tin) ? 1.0 : -1.0;

    // capstan target
    double Tbias = Tin * exp(sigma * mu * phi);

    // propose update
    double Ttrial = Tbias;

    // compute violation
    double violation = 0.0;
    if (Ttrial < Tmin)
        violation = (Tmin - Ttrial) / Tmin;
    else if (Ttrial > Tmax)
        violation = (Ttrial - Tmax) / Tmax;

    // alpha mapping
    double alpha_scale = this->dataPtr->alpha_scale;
    double alpha = 1.0 - std::exp(-violation / alpha_scale);

    // blend
    double Tblend = (1.0 - alpha) * Told + alpha * Ttrial;

    // project
    double Tout = std::clamp(Tblend, Tmin, Tmax);

    this->dataPtr->T_out[i] = Tout;
    this->dataPtr->T_in[i+1] = Tout;

    // direction for wrench
    math::Vector3d u12 = this->dataPtr->segmentDirs[i];
    math::Vector3d f12 = Tout * u12;
    math::Vector3d f21 = -f12;

    segment.link1.AddWorldWrench(_ecm, f12, math::Vector3d::Zero, segment.site1);
    segment.link2.AddWorldWrench(_ecm, f21, math::Vector3d::Zero, segment.site2);
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
