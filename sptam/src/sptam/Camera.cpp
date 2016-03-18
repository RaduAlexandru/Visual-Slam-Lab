/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#include "Camera.hpp"

Camera::Camera(const CameraPose& pose, const CameraParameters& calibration)
  : pose_( pose ), calibration_( calibration )
  , frustum_(
      pose_,
      calibration.horizontalFOV, calibration.verticalFOV,
      calibration.frustumNearPlaneDist, calibration.frustumFarPlaneDist
    )
{}

void Camera::UpdatePose(const CameraPose& newPose)
{
  pose_ = newPose;

  frustum_ = FrustumCulling(
    pose_,
    calibration_.horizontalFOV, calibration_.verticalFOV,
    calibration_.frustumNearPlaneDist, calibration_.frustumFarPlaneDist
  );
}

std::ostream& operator << ( std::ostream& os, const Camera& camera)
{
  return os << camera.GetPose();
}
