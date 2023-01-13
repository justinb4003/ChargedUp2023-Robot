// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;
import frc.robot.subsystems.SwerveDriveTrain;

public class AlignToBar extends CommandBase {
  boolean finished = false;
  double kP = 0.017;
  double tolerance = 1;
  long stopTime;
  /** Creates a new AlignToTarget. */
  public AlignToBar() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopTime = System.currentTimeMillis() + 1500;
    finished = false;
  }
  double powerThreshold = .10;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    
    double currentAngle = currentPose.getRotation().getDegrees();
    //System.out.println(currentAngle);
    double error = currentAngle - 180;
    if(Math.abs(error) < 3){
      finished = true;
    }
    while(error > 180){
      error -= 360;

    }
    while(error < -180){
      error += 360;
    }
    double rotationPower = -2*error/180 *RobotData.maxAngularSpeed;


    RobotContainer.swerveDrive.drive(0, 0, rotationPower);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || System.currentTimeMillis() > stopTime;
  }
}
