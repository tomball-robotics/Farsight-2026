// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.T3Lib;

public class IntakePivot extends SubsystemBase {

TalonFX leader;
TalonFX follower;

public IntakePivot() {
  leader = new TalonFX(Constants.IntakePivotConstants.INTAKE_PIVOT_LEADER_ID);
  follower = new TalonFX(Constants.IntakePivotConstants.INTAKE_PIVOT_FOLLOWER_ID);
  TalonFXConfiguration config = new TalonFXConfiguration();

  config.Slot0.kP = .7;
  config.Slot0.kI = 0.001;
  config.Slot0.kD = 0;


  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  T3Lib.applyConfig(leader, config);
  T3Lib.applyConfig(follower, config);
  

  follower.setControl(new Follower(Constants.IntakePivotConstants.INTAKE_PIVOT_LEADER_ID, MotorAlignmentValue.Opposed));
}

 public Command toPosition(double position){
  return runOnce(() -> {
     System.out.println(position);
     leader.setControl(new PositionVoltage(position).withSlot(0));
   });
}



public Command dropIntake(){
  return run(() -> {
    if(leader.getPosition().getValueAsDouble() < Constants.IntakePivotConstants.DOWN_POSITION/2){
      System.out.println("Down");
      leader.setControl(new PositionVoltage(Constants.IntakePivotConstants.DOWN_POSITION).withSlot(0));
    }
    else{
        System.out.println("Up");

      leader.setControl(new CoastOut());
    }

  });
}
public Command bringUpIntake(){
  return toPosition(Constants.IntakePivotConstants.UP_POSITION);
}

public Command ifNotDownPutDown(){
  return Commands.either(
    dropIntake(),
    Commands.none(),
    () -> Math.abs(leader.getPosition().getValueAsDouble() - Constants.IntakePivotConstants.DOWN_POSITION) >= 0.02
  );
}
  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Extended", Math.abs(leader.getPosition().getValueAsDouble() - Constants.IntakePivotConstants.DOWN_POSITION) < 0.02);
    SmartDashboard.putNumber("Intake/Position Leader", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Applied Output", leader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Position Follower", follower.getPosition().getValueAsDouble());

  }
}
