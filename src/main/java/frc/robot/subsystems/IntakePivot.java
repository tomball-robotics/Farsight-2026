// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
TalonFX motor;
final PositionVoltage request = new PositionVoltage(0).withSlot(0);
public IntakePivot() {
  motor = new TalonFX(Constants.IntakePivotConstants.INTAKE_PIVOT_ID);
  TalonFXConfiguration config = new TalonFXConfiguration();

  config.Slot0.kP = 0.01;
  config.Slot0.kI = 0;
  config.Slot0.kD = 0;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  motor.getConfigurator().apply(config);
}

public Command ToPosition(double position){
  return runOnce(() -> {
    motor.setControl(request.withPosition(position));
  });
}

public Command dropIntake(){return ToPosition(Constants.IntakePivotConstants.DOWN_POSITION);}
public Command bringUpIntake(){return ToPosition(0);}

  @Override
  public void periodic() {
    
  }
}
