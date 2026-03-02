// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.T3Lib;

public class Climber extends SubsystemBase {
  TalonFX motor;
  public Climber() {
    motor = new TalonFX(Constants.ClimberConstants.CLIMBER_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.01;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.TOP_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    T3Lib.applyConfig(motor, config);
  }

  public Command runClimber(double velocity){
    return runOnce(() -> motor.setControl(new VoltageOut(velocity)));
  }

  public Command climberUp(){return runClimber(Constants.ClimberConstants.CLIMBER_SPEED);}
  public Command climberDown(){return runClimber(-Constants.ClimberConstants.CLIMBER_SPEED);}

  public Command stop(){return runOnce(() -> motor.setControl(new NeutralOut()));}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Climber Position", motor.getPosition().getValueAsDouble());
  }
}
