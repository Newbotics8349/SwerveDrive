// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import javax.swing.tree.TreeNode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/** Add your docs here. */
public class VisionIO extends SubsystemBase {
    // Object camera;


    public void init(String cam_name){
        
    }

    public Pose3d get_vision_pose(){
        return new Pose3d();
    }

    public Optional<Pose3d> get_pose_apriltag_relative(){
  
        return Optional.empty();
    }

    public double get_timestamp(){
        return 0;
    }

    @Override
    public void periodic(){

    }
}
