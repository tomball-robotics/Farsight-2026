// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.lib.T3Lib;

public class Climber extends SubsystemBase {
 
  TalonFX rightClimberMotor;
  TalonFX leftClimberMotor;

  TalonFXConfiguration config = new TalonFXConfiguration();

  PositionVoltage request = new PositionVoltage(0);

  public Climber() {
    rightClimberMotor = T3Lib.createTalonFXPosition(Constants.ClimberConstants.RIGHT_CLIMBER_ID, NeutralModeValue.Brake, false, Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT, 0.01, 0, 0);
    leftClimberMotor = T3Lib.createTalonFXPosition(Constants.ClimberConstants.LEFT_CLIMBER_ID, NeutralModeValue.Brake, false, Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT, 0.01, 0, 0);

    
    leftClimberMotor.setControl(new Follower(Constants.ClimberConstants.RIGHT_CLIMBER_ID, MotorAlignmentValue.Opposed));

  }

  public Command runClimber(double velocity){
    return runOnce(() -> rightClimberMotor.setControl(new VoltageOut(velocity)));
  }

  public Command climberUp(){return runClimber(Constants.ClimberConstants.CLIMBER_SPEED);}
  public Command climberDown(){return runClimber(-Constants.ClimberConstants.CLIMBER_SPEED);}

  public Command autoClimberUp(){return run(() -> rightClimberMotor.setControl(request.withPosition(Constants.ClimberConstants.CLIMBER_EXTENDED_POSITION).withSlot(0)))
    .until(() -> Math.abs(Constants.ClimberConstants.CLIMBER_EXTENDED_POSITION - rightClimberMotor.getPosition().getValueAsDouble()) < 0.5).andThen(() ->
    rightClimberMotor.setControl(new NeutralOut()));
  }
  public Command autoClimberDown(){return run(() -> rightClimberMotor.setControl(request.withPosition(Constants.ClimberConstants.CLIMBER_DOWN_POSITION).withSlot(0)))
    .until(() -> Math.abs(rightClimberMotor.getPosition().getValueAsDouble() - Constants.ClimberConstants.CLIMBER_DOWN_POSITION) < 0.5).andThen(() ->
    rightClimberMotor.setControl(new NeutralOut()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(rightClimberMotor.getPosition().getValueAsDouble() <= 0.1 && rightClimberMotor.getVelocity().getValueAsDouble() <= 0){rightClimberMotor.setControl(new VoltageOut(0));}
    if(rightClimberMotor.getPosition().getValueAsDouble() >= ClimberConstants.MAX_EXTENSION && rightClimberMotor.getVelocity().getValueAsDouble() >= 0){rightClimberMotor.setControl(new VoltageOut(0));}
  }
}