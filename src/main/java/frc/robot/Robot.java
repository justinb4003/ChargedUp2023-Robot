// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetOdometry;
import frc.robot.subsystems.SwerveDriveTrain;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private final XboxController m_controller = new XboxController(0);
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.startLogger();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //RobotContainer.swerveDrive.updateOdometry();
  }
  public void autonomousInit(){
    RobotContainer.swerveDrive.setFieldRelative(true);
    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
  //(new SetOdometry(250, 30, 180)).schedule();
  }

  @Override
  public void autonomousPeriodic() {
  //  RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
  }

}
