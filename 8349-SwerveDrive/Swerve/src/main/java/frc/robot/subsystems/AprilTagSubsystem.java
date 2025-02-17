// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.dyn4j.geometry.Transform;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagSubsystem extends SubsystemBase {
  private PhotonCamera camera;
  private List<PhotonTrackedTarget> tagsTracked = new ArrayList<PhotonTrackedTarget>();

  public AprilTagSubsystem(String cameraNameString) {
    camera = new PhotonCamera(cameraNameString);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Transform3d getTransform()
  {
    if (!tagsTracked.isEmpty())
    {
      return tagsTracked.get(0).getBestCameraToTarget();
    }
    return new Transform3d();
  }

  public boolean hasTargets() {
    return tagsTracked.size() > 0;
  }

  @Override
  public void periodic() {
    // Look for new targets
    tagsTracked.clear();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() > 0)
    {
      PhotonPipelineResult lastResult = results.get(0);
      for (PhotonTrackedTarget target : lastResult.targets)
      {
        tagsTracked.add(target);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
