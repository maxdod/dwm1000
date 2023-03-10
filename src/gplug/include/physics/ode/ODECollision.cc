/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
/* Desc: ODECollision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODECollision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODECollision::ODECollision(LinkPtr _link)
: Collision(_link)
{
  this->SetName("ODE_Collision");
  this->collisionId = nullptr;
  this->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;

  this->SetSpaceId(
      boost::static_pointer_cast<ODELink>(this->link)->GetSpaceId());

  this->surface.reset(new ODESurfaceParams());
}

//////////////////////////////////////////////////
ODECollision::~ODECollision()
{
  if (this->collisionId)
    dGeomDestroy(this->collisionId);
  this->collisionId = nullptr;

  this->Fini();
}

//////////////////////////////////////////////////
void ODECollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }

  // Force max correcting velocity to zero for certain collision entities
  if (this->IsStatic() || this->shape->HasType(Base::HEIGHTMAP_SHAPE) ||
      this->shape->HasType(Base::MAP_SHAPE))
  {
    this->GetODESurface()->maxVel = 0.0;
  }

  // Check validity of plowing parameters
  const std::string kPlowingWheel = "gz:plowing_wheel";
  sdf::ElementPtr wheelElem = nullptr;
  if (_sdf->HasElement(kPlowingWheel))
  {
    wheelElem = _sdf->GetElement(kPlowingWheel);
  }

  // Try to parse wheel plowing params with verbose == true so errors will
  // print once during loading
  ODECollisionWheelPlowingParams plowing;
  ParseWheelPlowingParams(_sdf, plowing, this->GetScopedName());
}

//////////////////////////////////////////////////
bool ODECollision::ParseWheelPlowingParams(
    sdf::ElementPtr _sdf,
    ODECollisionWheelPlowingParams &_plowing,
    const std::string &_scopedNameForErrorMessages)
{
  // Only print error messages of the scoped name is supplied.
  bool verbose = !_scopedNameForErrorMessages.empty();

  // Check for a plowing wheel element
  const std::string kPlowingWheel = "gz:plowing_wheel";
  if (!_sdf->HasElement(kPlowingWheel))
  {
    return false;
  }
  sdf::ElementPtr wheelElem = _sdf->GetElement(kPlowingWheel);

  // Check for required elements: max_degrees and saturation velocity
  const std::string kPlowingMaxDegrees = "max_degrees";
  if (!wheelElem->HasElement(kPlowingMaxDegrees))
  {
    if (verbose)
    {
      gzerr << "Element <" << kPlowingWheel << "> in collision with name ["
            << _scopedNameForErrorMessages << "] is missing required parameter"
            << " <" << kPlowingMaxDegrees << ">" << std::endl;
    }
    return false;
  }

  const std::string kPlowingSaturationVelocity = "saturation_velocity";
  if (!wheelElem->HasElement(kPlowingSaturationVelocity))
  {
    if (verbose)
    {
      gzerr << "Element <" << kPlowingWheel << "> in collision with name ["
            << _scopedNameForErrorMessages << "] is missing required parameter"
            << " <" << kPlowingSaturationVelocity << ">" << std::endl;
    }
    return false;
  }

  // Error unless max_degrees >= 0
  double plowingMaxDegrees = wheelElem->Get<double>(kPlowingMaxDegrees);
  if (!(plowingMaxDegrees >= 0))
  {
    if (verbose)
    {
      gzerr << "Element <" << kPlowingWheel << "> in collision with name ["
            << _scopedNameForErrorMessages << "] has a parameter "
            << "<" << kPlowingMaxDegrees << "> with a value of ["
            << plowingMaxDegrees << "] that should be non-negative."
            << std::endl;
    }
    return false;
  }
  ignition::math::Angle plowingMaxAngle;
  plowingMaxAngle.SetDegree(plowingMaxDegrees);

  // Error unless saturation_velocity > 0
  double plowingSaturationVelocity =
      wheelElem->Get<double>(kPlowingSaturationVelocity);
  if (!(plowingSaturationVelocity > 0))
  {
    if (verbose)
    {
      gzerr << "Element <" << kPlowingWheel << "> in collision with name ["
            << _scopedNameForErrorMessages << "] has a parameter "
            << "<" << kPlowingSaturationVelocity << "> with a value of ["
            << plowingSaturationVelocity << "] that should be positive."
            << std::endl;
    }
    return false;
  }

  // Check also for optional element: deadband velocity
  const std::string kPlowingDeadbandVelocity = "deadband_velocity";
  // Use deadband velocity = 0 if parameter is not set
  double plowingDeadbandVelocity =
      wheelElem->Get<double>(kPlowingDeadbandVelocity, 0.0).first;

  // Error unless deadband velocity < saturation velocity
  if (!(plowingDeadbandVelocity < plowingSaturationVelocity))
  {
    if (verbose)
    {
      gzerr << "Element <" << kPlowingWheel << "> in collision with name ["
            << _scopedNameForErrorMessages << "] has a parameter "
            << "<" << kPlowingDeadbandVelocity << "> with a value of ["
            << plowingDeadbandVelocity << "] that should be less than the "
            << "<" << kPlowingSaturationVelocity << "> whose value is ["
            << plowingSaturationVelocity << "]."
            << std::endl;
    }
    return false;
  }

  _plowing.maxAngle = plowingMaxAngle;
  _plowing.saturationVelocity = plowingSaturationVelocity;
  _plowing.deadbandVelocity = plowingDeadbandVelocity;
  return true;
}

//////////////////////////////////////////////////
void ODECollision::Fini()
{
  /*
     if (this->collisionId)
     dGeomDestroy(this->collisionId);
     this->collisionId = nullptr;

     if (this->spaceId)
     dSpaceDestroy(this->spaceId);
     this->spaceId = nullptr;
     */

  Collision::Fini();
}

//////////////////////////////////////////////////
void ODECollision::OnPoseChange()
{
  // Update all the models
  // (*this.*onPoseChangeFunc)();

  if (this->IsStatic() && this->collisionId && this->placeable)
    this->OnPoseChangeGlobal();
  else if (this->collisionId && this->placeable)
    this->OnPoseChangeRelative();
}

//////////////////////////////////////////////////
void ODECollision::SetCollision(dGeomID _collisionId, bool _placeable)
{
  // Must go first in this function
  this->collisionId = _collisionId;

  Collision::SetCollision(_placeable);

  if (dGeomGetSpace(this->collisionId) == 0)
  {
    dSpaceAdd(this->spaceId, this->collisionId);
    GZ_ASSERT(dGeomGetSpace(this->collisionId) != 0, "Collision ID is null");
  }

  if (this->collisionId && this->placeable)
  {
    if (this->IsStatic())
      this->onPoseChangeFunc = &ODECollision::OnPoseChangeGlobal;
    else
      this->onPoseChangeFunc = &ODECollision::OnPoseChangeRelative;
  }
  else
  {
    this->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;
  }

  dGeomSetData(this->collisionId, this);
}

//////////////////////////////////////////////////
dGeomID ODECollision::GetCollisionId() const
{
  return this->collisionId;
}

//////////////////////////////////////////////////
int ODECollision::GetCollisionClass() const
{
  int result = 0;

  if (this->collisionId)
  {
    result = dGeomGetClass(this->collisionId);
  }

  return result;
}

//////////////////////////////////////////////////
void ODECollision::SetCategoryBits(unsigned int _bits)
{
  if (this->collisionId)
    dGeomSetCategoryBits(this->collisionId, _bits);
  if (this->spaceId)
    dGeomSetCategoryBits((dGeomID)this->spaceId, _bits);
}

//////////////////////////////////////////////////
void ODECollision::SetCollideBits(unsigned int _bits)
{
  if (this->collisionId)
    dGeomSetCollideBits(this->collisionId, _bits);
  if (this->spaceId)
    dGeomSetCollideBits((dGeomID)this->spaceId, _bits);
}

//////////////////////////////////////////////////
ignition::math::AxisAlignedBox ODECollision::BoundingBox() const
{
  dReal aabb[6];
  memset(aabb, 0, 6 * sizeof(dReal));

  // if (this->collisionId && this->type != Shape::PLANE)
  dGeomGetAABB(this->collisionId, aabb);

  ignition::math::AxisAlignedBox box(
      ignition::math::Vector3d(aabb[0], aabb[2], aabb[4]),
      ignition::math::Vector3d(aabb[1], aabb[3], aabb[5]));

  return box;
}

//////////////////////////////////////////////////
dSpaceID ODECollision::GetSpaceId() const
{
  return this->spaceId;
}

//////////////////////////////////////////////////
void ODECollision::SetSpaceId(dSpaceID _spaceid)
{
  this->spaceId = _spaceid;
}

/////////////////////////////////////////////////
ODESurfaceParamsPtr ODECollision::GetODESurface() const
{
  return boost::dynamic_pointer_cast<ODESurfaceParams>(this->surface);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeGlobal()
{
  // A collision is not guaranteed to have a link (such as a standalone ray)
  if (!this->link)
    return;

  dQuaternion q;

  // Transform into global pose since a static collision does not have a link
  ignition::math::Pose3d localPose = this->WorldPose();

  // un-offset cog location
  ignition::math::Vector3d cogVec = this->link->GetInertial()->CoG();
  localPose.Pos() = localPose.Pos() - cogVec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  dGeomSetPosition(this->collisionId, localPose.Pos().X(),
      localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetQuaternion(this->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeRelative()
{
  // A collision is not guaranteed to have a link (such as a standalone ray)
  if (!this->link)
    return;

  dQuaternion q;
  // Transform into CoM relative Pose
  ignition::math::Pose3d localPose = this->RelativePose();

  // un-offset cog location
  ignition::math::Vector3d cogVec = this->link->GetInertial()->CoG();
  localPose.Pos() = localPose.Pos() - cogVec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  // Set the pose of the encapsulated collision; this is always relative
  // to the CoM
  dGeomSetOffsetPosition(this->collisionId,
      localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetOffsetQuaternion(this->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeNull()
{
}
