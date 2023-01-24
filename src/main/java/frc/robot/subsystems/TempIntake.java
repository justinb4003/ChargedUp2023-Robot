// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TempIntake extends SubsystemBase {
  VictorSPX intakeMotor;
  VictorSPX intakeMotor2;
  /** Creates a new TempIntake. */
  public TempIntake() {
    intakeMotor = new VictorSPX(4);
    intakeMotor2 = new VictorSPX(5);
    intakeMotor2.setInverted (false);
  }

  public void setPowerOn(boolean on) {
    System.out.println("Seting temp intake: " + on);
    if(on) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.8);
      intakeMotor2.set(VictorSPXControlMode.PercentOutput, 0.8);
    } else {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      intakeMotor2.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
  }

  public void backOut() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, -0.8);
    intakeMotor2.set(VictorSPXControlMode.PercentOutput, -0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
