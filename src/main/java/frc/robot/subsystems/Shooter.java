// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX velocityMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_VELOCITY_MOTOR_ID);
  TalonFX angleMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_ANGLE_MOTOR_ID);
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);


  public Shooter() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.01;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotor.getConfigurator().apply(config);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    velocityMotor.getConfigurator().apply(config);
  }

  public Command setVelocity(double mps){
    return runOnce(() -> velocityMotor.setControl(new VoltageOut(mps * Constants.ShooterConstants.VOLTS_TO_MPS)));
  }

  public Command setAngle(double degrees){
    return runOnce(() -> angleMotor.setControl(request.withPosition(degrees * Constants.ShooterConstants.TICKS_TO_DEGREES)));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
