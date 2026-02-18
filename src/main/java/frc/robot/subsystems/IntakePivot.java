// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
TalonFX leader;
TalonFX follower;
final PositionVoltage request = new PositionVoltage(0).withSlot(0);
public IntakePivot() {
  leader = new TalonFX(Constants.IntakePivotConstants.INTAKE_PIVOT_LEADER_ID);
  follower = new TalonFX(Constants.IntakePivotConstants.INTAKE_PIVOT_FOLLOWER_ID);
  TalonFXConfiguration config = new TalonFXConfiguration();

  config.Slot0.kP = 0.01;
  config.Slot0.kI = 0;
  config.Slot0.kD = 0;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  leader.getConfigurator().apply(config);
  follower.getConfigurator().apply(config);
  follower.setControl(new Follower(Constants.IntakePivotConstants.INTAKE_PIVOT_LEADER_ID, MotorAlignmentValue.Opposed));
}

public Command ToPosition(double position){
  return run(() -> {
    leader.setControl(request.withPosition(position));
  }).until(() -> Math.abs(leader.getPosition().getValueAsDouble() - position) < 0.02);
}

public Command dropIntake(){return ToPosition(Constants.IntakePivotConstants.DOWN_POSITION);}
public Command bringUpIntake(){return ToPosition(0);}

public Command ifNotDownPutDown(){
  return runOnce(() -> {
    if(!(Math.abs(leader.getPosition().getValueAsDouble() - Constants.IntakePivotConstants.DOWN_POSITION) < 0.02)){
      dropIntake();
    }});
}

  @Override
  public void periodic() {
    
  }
}
