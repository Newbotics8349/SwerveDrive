// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class vision_tag_response {
    Optional<Transform3d> camera_to_target_pose = Optional.empty();
    AprilTagFieldLayout aprilTagFieldLayout =AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    int tag_id;

    public vision_tag_response(Transform3d camera_to_target_pose, int tag_id){
        this.camera_to_target_pose = Optional.of(camera_to_target_pose);
        this.tag_id = tag_id;
    }

    public Pose3d get_robot_pose_relative(){
        Pose3d tagPose = this.aprilTagFieldLayout.getTagPose(tag_id).get();
        tagPose = tagPose.transformBy(camera_to_target_pose.get());
        return tagPose;
    }
}
