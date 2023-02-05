// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  final double INCHESPERMETER = 39.36;
  double tx, ty, tv;
  double[] pose; 
  /** Creates a new Limelight. */
  public Limelight() {}

  public boolean isTargetValid(){
    return tv > .5;
  }
  public double getTargetX(){
    return tx;
  }
  public double getTargetY(){
    return ty;
  }
  public double getRobotX(){
    //double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{});
    return pose[0] * INCHESPERMETER;
  }
  public double getRobotY(){
    //double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{});
    return pose[1] * INCHESPERMETER;
  }
  public double getRobotRotation(){
   // double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{});
    return pose[5];
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* 
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("tx", tx);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("ty", ty);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    SmartDashboard.putNumber("tv", tv);
    */
    double[] newPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{1234});
    if(newPose.length < 6) return;
    pose = newPose;
    //System.out.println(pose[0] + " " + pose[1] + " " + pose[5]);
  }

}
