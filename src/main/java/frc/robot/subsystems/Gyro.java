// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private final AHRS gyro = new AHRS(Port.kUSB1);
  Rotation2d initialHeading = new Rotation2d();
  double offsetX = 0;
  double offsetY = 0;

  public Gyro() {}

  public void setInitialHeading(double heading) {
    initialHeading = Rotation2d.fromDegrees(heading);
  }

  public void reset() {
    gyro.reset();
  }

  public double getYaw() {
   return gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getPitch();
  }
  public double getRoll() {
    return gyro.getRoll();
  }
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d().plus(initialHeading);
  }

  public double getVelocityX() {
    return gyro.getVelocityX();
  }

  public double getVelocityY() {
    return gyro.getVelocityY();
  }

  public double getVelocityZ() {
    return gyro.getVelocityZ();
  }

  public void resetDisplacement(){
    offsetX = gyro.getDisplacementX();
    offsetY = gyro.getDisplacementY();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(gyro.getRotation2d());
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
  }
}
