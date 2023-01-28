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
  boolean powerOn = false;
  /** Creates a new TempIntake. */
  public TempIntake() {
    intakeMotor = new VictorSPX(40);
    intakeMotor2 = new VictorSPX(50);
    intakeMotor2.setInverted(true);
  }
  public void togglePower(){
    powerOn = ! powerOn;
  }
  public void setPowerOn() {
    System.out.println("Seting temp intake: " + powerOn);
    if(powerOn) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
      intakeMotor2.set(VictorSPXControlMode.PercentOutput, 1);
    } else {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      intakeMotor2.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
  }
  public void setPower(double power){
    intakeMotor.set(VictorSPXControlMode.PercentOutput, power);
    intakeMotor2.set(VictorSPXControlMode.PercentOutput, power);
  }
  public void backOut() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, -0.8);
    intakeMotor2.set(VictorSPXControlMode.PercentOutput, -0.8);
  }

  @Override
  public void periodic() {
   //This method will be called once per scheduler run
  }
}
