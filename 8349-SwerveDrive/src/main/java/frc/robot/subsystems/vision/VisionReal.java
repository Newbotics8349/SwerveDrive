// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionReal extends VisionIO {

  public VisionReal(){}

  PhotonCamera camera;
  PhotonPipelineResult result;
  PhotonPoseEstimator pose_Estimator;
  
  //Change to support more cameras in future. Put in constructor and pass transformation as parameter
  Transform3d camPose = Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM;

  @Override
  public void init(String cam_name){
    this.camera = new PhotonCamera(cam_name);

    this.pose_Estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camPose);
    this.pose_Estimator.setRobotToCameraTransform(this.camPose);

    this.result = camera.getLatestResult();
  }
  @Override
  public Pose3d get_vision_pose(){
    Optional<EstimatedRobotPose> pose = pose_Estimator.update();

    if (pose.isPresent()){
      return pose.get().estimatedPose;
    }
    else{
      return new Pose3d();
    }
  }

  @Override
  public double get_timestamp(){
    return this.result.getTimestampSeconds();
  }

  @Override
  public void periodic() {
    this.result = camera.getLatestResult();
    
    // This method will be called once per scheduler run
  }
}
