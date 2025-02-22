// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.lang.annotation.Target;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagSubsystem extends SubsystemBase {
  private PhotonCamera camera;
  private List<PhotonTrackedTarget> tagsTracked = new ArrayList<PhotonTrackedTarget>();

  public AprilTagSubsystem(String cameraNameString) {
    camera = new PhotonCamera(cameraNameString);
  }

  private PhotonTrackedTarget getTagById(int tagId) {
    for (PhotonTrackedTarget target : tagsTracked) {
      if (target.getFiducialId() == tagId)
        return target;
    }
    return null;
  }

  public List<PhotonTrackedTarget> getTargets() {
    return tagsTracked;
  }

  public PhotonTrackedTarget getBestTarget(List<PhotonTrackedTarget> targetList) {
    PhotonTrackedTarget bestTarget = null;
    for (PhotonTrackedTarget target : targetList) {
      if (bestTarget == null) {
        bestTarget = target;
      } else {
        if (Math.abs(bestTarget.getYaw()) > Math.abs(target.getYaw())) {
          bestTarget = target;
        }
      }
    } 
    return bestTarget;
  }

  public Pose2d getCameraToTagPose(int tagId) {
    // Get tag object if it exists
    PhotonTrackedTarget target = getTagById(tagId);
    if (target == null)
      return null;

    return getCameraToTagPose(target);
  }

  public Pose2d getCameraToTagPose(PhotonTrackedTarget target) {
    // Get transform from target
    Transform3d transform3d = target.getBestCameraToTarget();

    // Offset the target transform considering that the camera is not in the center
    // of the robot
    transform3d.plus(new Transform3d(Constants.cameraOnRobot));

    // Create a Pose2d from the Transform3d
    return new Pose2d(transform3d.getX(), transform3d.getY(), transform3d.getRotation().toRotation2d());
  }

  public boolean hasTargets() {
    return tagsTracked.size() > 0;
  }

  @Override
  public void periodic() {
    // Look for new targets
    tagsTracked.clear();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      PhotonPipelineResult lastResult = results.get(0);
      for (PhotonTrackedTarget target : lastResult.targets) {
        tagsTracked.add(target);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
