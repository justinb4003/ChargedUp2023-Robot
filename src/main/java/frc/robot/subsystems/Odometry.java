// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Odometry extends SubsystemBase {
  private Pose2d pose;
  final double INCHESPERMETER = 39.36;

  /** Creates a new Odometry. */
  public Odometry() {}

  public Pose2d getPose(){
    return pose;
  }

  @Override
  public void periodic() {
    double[] newPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{10000});
    // System.out.println(newPose[0]);
    boolean aprilTagOk = false;
    if(newPose.length < 6 || (newPose[0] == 0.0 && newPose[1] == 0)){
      if (pose != null){
        double deltaX = newPose[0] - pose.getX();
        double deltaY = newPose[1] - pose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double velocity = distance / 0.02;
        if (velocity < RobotContainer.swerveDrive.kMaxSpeed){
          aprilTagOk = true;
        }
        
      }
      else {
        aprilTagOk = true;
      }
    }
    if (false) {
      pose = new Pose2d(newPose[0] * INCHESPERMETER, newPose[1] * INCHESPERMETER, Rotation2d.fromDegrees(newPose[5]));
      RobotContainer.swerveDrive.setOdometry(pose);
    }
    else{
      pose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    }
    
    
    // pose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    SmartDashboard.putNumber("X Location", pose.getX());
    SmartDashboard.putNumber("Y Location", pose.getY());
    SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees());
  }
}
