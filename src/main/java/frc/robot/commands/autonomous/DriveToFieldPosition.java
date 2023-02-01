// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveTrain;

public class DriveToFieldPosition extends CommandBase {
  public double targetT;
  public double targetS;
  public double deltaAngle;
  public double distance, rotation, deltaX, deltaY;
  public boolean finished = false;
  public double lastDistance = 10000;
  double aprilTagX = 7.2431 * 39.36;
  double aprilTagY = -1.26019 * 39.36;
  /** Creates a new DriveToFieldPosition. */
  public DriveToFieldPosition(double t , double s) {
    targetT = t;
    targetS = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setOdometry(100, -20, 0);
    RobotContainer.swerveDrive.setFieldRelative(false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = RobotContainer.limelight.getRobotX();
    double y = RobotContainer.limelight.getRobotY();
    rotation = RobotContainer.limelight.getRobotRotation();
    //Pose2d location = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    //x = location.getX();
    //y = location.getY();
    deltaX = targetT - x;
    deltaY = targetS - y;
    distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    double angleRad = Math.atan2(deltaY, deltaX);
    double angleDeg = Math.toDegrees(angleRad);
    deltaAngle = angleDeg - rotation;
    while (deltaAngle < -180) deltaAngle = deltaAngle + 360;
    while (deltaAngle > 180) deltaAngle = deltaAngle - 360;
    deltaAngle = Math.toRadians(deltaAngle);

    deltaX = aprilTagX - x;
    deltaY = aprilTagY - y;
    double aprilTagAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
    double deltaRotation = aprilTagAngle - rotation + 180;
    while (deltaRotation < -180) deltaRotation = deltaRotation + 360;
    while (deltaRotation > 180) deltaRotation = deltaRotation - 360;

    // set up power readings
    double rampDistance = distance;
    double maxPower = 0.25;
    double driveFactor = 1;
    double rotFactor = 0;
    double finalRamp = Math.min(1, distance/rampDistance);
    double drivePower = maxPower * driveFactor * finalRamp;
    double ySpeed = Math.sin(deltaAngle) * drivePower * SwerveDriveTrain.kMaxSpeed;
    double xSpeed = Math.cos(deltaAngle) * drivePower * SwerveDriveTrain.kMaxSpeed;
    double rotationSpeed = deltaRotation / 180 * SwerveDriveTrain.kMaxAngularSpeed;

    // set drive power
    //System.out.println(xSpeed + " " + ySpeed + " " + rotationSpeed);
    //System.out.println(distance + " " + lastDistance);
    RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rotationSpeed);
    SmartDashboard.putNumber("distance to target", distance);
    SmartDashboard.putNumber("Robot X", x);
    SmartDashboard.putNumber("Robot Y", y);
    // check to see if we're finished
    finished = distance < 20 && distance > lastDistance;
    lastDistance = distance;

    // update ramp up 
    driveFactor += 0.1;
    rotFactor += 0.1;
    if (driveFactor > 1) driveFactor = 1;
    if (rotFactor > 1) rotFactor = 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
