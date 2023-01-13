// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignToTarget(),
      new DriveToPose(-66, -5, 0, 0.4),
      new AlignToTarget(),
      new DriveToPose(-3, 149, -45, 0.4),
      new AlignToTarget()
    );
  }
}
