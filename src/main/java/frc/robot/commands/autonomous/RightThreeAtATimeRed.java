// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightThreeAtATimeRed extends SequentialCommandGroup {

  double[][] first_waypoints = new double[][] {
    {298, 28},
    {288.76391294623943, 57.37171717825354},
    //{252.66842382795951, 86.20941466523873},
    //{200, 72}
    {253, 82},
    {200, 68}
  };
  double[] first_headings = new double[] {45, 20, Math2d.goalAngle(first_waypoints[first_waypoints.length-1])};
  //this works
  /*double[][] second_waypoints = new double[][] {
    {206.54110500358627, 79.7277389337725},
    {148.5384549185883, 86},
    {86.23872279569983, 72},   
    {24,48}
  
  }; */
  
  double[][] second_waypoints = new double[][] {
    {206.54110500358627, 79.7277389337725},
    {148.5384549185883, 82},
    {86,63},
    //{86.23872279569983, 70},   
    //{24,48}
    //{41, 46}
    //{43, 48}
    {54, 38}

  };
  double[] second_headings = new double[] {30, 45, 45};

  /** Creates a new RightFiveBall2. */
  public RightThreeAtATimeRed() {
    int shot1 = ShotData.FEET12;
    int shot2 = ShotData.FEET14;
    int shot3 = ShotData.FEET10;

    addCommands(new SetOdometry(296, 72, 90),
      new SetIntake(true),
      new DriveToAlignedPose(first_waypoints[0][0], first_waypoints[0][1], 0.35),
      new Wait(100),
      new SetShooterSpeed(ShotData.speeds[shot2]),
      new DriveSwerveProfile4(first_waypoints, first_headings, 0.4),
      new SetShot(shot2, true),
      new Wait(800),
      new SetShot(shot2, true),
      new Wait(800),
      new SetShot(shot2, true),
      new Wait(100),
      new SetShooter(0),
      new DriveSwerveProfile4(second_waypoints, second_headings, 0.5),
      //new Wait(2000),

      new Wait(1000),
      new DriveForDistance(20, new double[]{1,-1}, 0.2),
      new Wait(800),

      /*
      new Wait(700),
      new SetJustIntake(false),
      new Wait(500),
      new SetIntake(true),
      new Wait(400),
      */

      new ParallelCommandGroup(
        new SequentialCommandGroup(new Wait(1500), new SetShooterSpeed(ShotData.speeds[shot3])),
        new DriveToAlignedPose(240, 60, 0.6, -14)
      ),
      new SetShot(shot3, true),
      new Wait(1500),
      new SetIntake(false),
      new SetShooter(0)
    );

  }
}