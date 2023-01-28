package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveModule;

public class RobotTelemetry {
    
    public static void monitorData() {
        SmartDashboard.putNumber("Gyro Roll", RobotContainer.gyro.getRoll());
        SmartDashboard.putNumber("Pose X", RobotContainer.swerveDrive.getOdometry().getPoseMeters().getX());
        SmartDashboard.putNumber("Pose Y", RobotContainer.swerveDrive.getOdometry().getPoseMeters().getY());
        for (int i = 0; i < RobotContainer.swerveDrive.getModules().length; i++) {
            SwerveModule module = RobotContainer.swerveDrive.getModules()[i];
            SmartDashboard.putNumber("Module " + i + " Distance", module.getState().distanceMeters);
            SmartDashboard.putNumber("Module " + i + " Velocity", module.getDriveVelocity());
        }
    }

}
