// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveTrain;

public class DriveUpRamp extends CommandBase {
  int state;
  final int DRIVEFORWARD = 0;
  final int DRIVEUPRAMP = 1;
  final int LEVELOFF = 2;
  final int VIBE = 3;
  final int END = 4;
  final int DRIVEDISTANCE = 5;
  double rollOffset; 
  int count = 0;
  double totalRoll;
  Pose2d initialPose;

  /** Creates a new DriveUpRamp. */
  public DriveUpRamp() {
    addRequirements(RobotContainer.swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    totalRoll = 0;
    state = DRIVEFORWARD;
    rollOffset = RobotContainer.gyro.getRoll();
    RobotContainer.swerveDrive.setFieldRelative(false);
  }

  double kp = 0.11 / 15;
  // double kp = 0.10 / 15;
  double ki = 0.0 / 75;
  double kd = kp * 8;
  double lastRoll;

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(state);
    double roll = -(RobotContainer.gyro.getRoll() - rollOffset);
    
    switch (state) {
      case DRIVEFORWARD: {
        RobotContainer.swerveDrive.drive(.3* SwerveDriveTrain.kMaxSpeed, 0, 0);
        if (roll > 5) state = DRIVEUPRAMP;
        break;
      }
      case DRIVEUPRAMP: {
        RobotContainer.swerveDrive.drive(0.40 * SwerveDriveTrain.kMaxSpeed, 0, 0);
        if (roll > 12){
           state = DRIVEDISTANCE;
           initialPose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
        }
        break;
      }
      case DRIVEDISTANCE:{
        Pose2d currentPose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
        double deltaX = currentPose.getX() - initialPose.getX();
        double deltaY = currentPose.getY() - initialPose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        //if (distance > 28) state = VIBE; 
        System.out.println(distance);
        double power = .4;
        if (distance > 20)power = .3;
        RobotContainer.swerveDrive.drive(power * SwerveDriveTrain.kMaxSpeed, 0, 0);
        break;
      }
      case VIBE: {
        totalRoll = roll + totalRoll * 0.6;
        double power = kp * roll + ki * totalRoll + kd * (roll - lastRoll);
       // if (roll < 5) state = LEVELOFF;
        RobotContainer.swerveDrive.drive(power * SwerveDriveTrain.kMaxSpeed, 0, 0);
        //if (roll < 1) state = END;
        break;
      }
      case LEVELOFF: {
        RobotContainer.swerveDrive.drive(-0.2 * SwerveDriveTrain.kMaxSpeed, 0, 0);
        if (count > 5) state = END;
        count++;
        break;
      }
    }
    lastRoll = roll;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == END;
  }
}
