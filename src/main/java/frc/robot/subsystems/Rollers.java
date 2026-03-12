// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.T3Lib;

public class Rollers extends SubsystemBase {
  TalonFX motor;
  double speed;
  
  public Rollers(int motorID, double speed) {

    this.speed = speed;
    motor = new TalonFX(motorID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    T3Lib.applyConfig(motor, config);
  }

  public Command run(int sign){
    return runOnce(() -> {
      motor.setControl(new VoltageOut(speed*sign));
    });
  }

  

  public Command stop(){
    return runOnce(() -> {
      motor.setControl(new NeutralOut());
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
