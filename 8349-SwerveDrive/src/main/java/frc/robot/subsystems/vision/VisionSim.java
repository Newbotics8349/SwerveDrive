// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.Console;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



public class VisionSim extends VisionIO {
  PhotonCameraSim cameraSim;
  VisionSystemSim visionSim;
  PhotonCamera photon_camera;
  PhotonPoseEstimator pose_Estimator;
  PhotonPipelineResult result;
  //Change to support more cameras in future. Put in constructor and pass transformation as parameter
  Transform3d camPose = Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  private LinearFilter filter_x_tag_relative = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter filter_y_tag_relative = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter filter_z_tag_relative = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter filter_r_tag_relative = LinearFilter.singlePoleIIR(0.1, 0.02);

  public VisionSim(){}

  @Override
  public void init(String cam_name){
    visionSim = new VisionSystemSim("camera_sim");

    visionSim.addAprilTags(aprilTagFieldLayout);
    
    var camera_properties = new SimCameraProperties();
    photon_camera = new PhotonCamera(cam_name);

    camera_properties.setCalibration(640, 360, Rotation2d.fromDegrees(90));
    camera_properties.setCalibError(0.35, 0.10);
    camera_properties.setFPS(70);
    camera_properties.setAvgLatencyMs(30);
    camera_properties.setLatencyStdDevMs(10);

    cameraSim = new PhotonCameraSim(photon_camera, camera_properties);

    visionSim.addCamera(cameraSim, camPose);
     
    cameraSim.enableDrawWireframe(true);
    result = photon_camera.getLatestResult();
  }

  @Override
  public Pose3d get_vision_pose(){
    return visionSim.getRobotPose();
  }

  @Override
  public Optional<Pose3d> get_pose_apriltag_relative(){
    Optional<Pose3d> output = Optional.empty();
    if (result.hasTargets()){

      vision_tag_response transformation = new vision_tag_response(result.getBestTarget().getBestCameraToTarget(), result.getBestTarget().getFiducialId());
      var x = filter_x_tag_relative.calculate(transformation.get_robot_pose_relative().getX());
      var y = filter_y_tag_relative.calculate(transformation.get_robot_pose_relative().getY());
      var z = filter_z_tag_relative.calculate(transformation.get_robot_pose_relative().getZ());
      var r = filter_r_tag_relative.calculate(transformation.get_robot_pose_relative().getRotation().getY());
      output = Optional.of(new Pose3d(new Translation3d(x, y, z), new Rotation3d(0,0,r)));
    }
    return output;
  }

  @Override 
  public double get_timestamp(){
    return result.getTimestampSeconds();
  }

  @Override
  public void periodic() {
    result = photon_camera.getLatestResult();
    visionSim.update(SwerveSubsystem.current_pose);
    // This method will be called once per scheduler run
  }
}
