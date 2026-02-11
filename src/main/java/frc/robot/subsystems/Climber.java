// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  TalonFX motor;
  public Climber() {
    motor = new TalonFX(Constants.ClimberConstants.CLIMBER_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
  }

  public Command runClimber(double velocity){
    return runOnce(() -> motor.setControl(new VoltageOut(velocity)));
  }

  public Command climberUp(){return runClimber(Constants.ClimberConstants.CLIMBER_SPEED);}
  public Command climberDown(){return runClimber(-Constants.ClimberConstants.CLIMBER_SPEED);}

  @Override
  public void periodic() {
    if(motor.getPosition().getValueAsDouble() <= 1 && motor.getVelocity().getValueAsDouble() <= 0){motor.setControl(new VoltageOut(0));}
    if(motor.getPosition().getValueAsDouble() >= ClimberConstants.MAX_LENGTH && motor.getVelocity().getValueAsDouble() >= 0){motor.setControl(new VoltageOut(0));}
  }
}
