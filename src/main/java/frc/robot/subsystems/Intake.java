package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib;

public class Intake extends SubsystemBase {
  
  TalonFX leader;
  TalonFX follower;
  TalonFX rollerMotor;
  
  public Intake() {
    leader = new TalonFX(Constants.IntakeConstants.INTAKE_PIVOT_LEADER_ID);
    follower = new TalonFX(Constants.IntakeConstants.INTAKE_PIVOT_FOLLOWER_ID);
    rollerMotor = new TalonFX(Constants.IntakeConstants.INTAKE_ROLLERS_ID);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = .7;
    pivotConfig.Slot0.kI = 0.001;
    pivotConfig.Slot0.kD = 0;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    T3Lib.applyConfig(leader, pivotConfig);
    follower.setControl(new Follower(Constants.IntakeConstants.INTAKE_PIVOT_LEADER_ID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    T3Lib.applyConfig(rollerMotor, rollerConfig);
    
  }
  
  public Command requestPosition(double position){
    return runOnce(() -> {
      System.out.println(position);
      leader.setControl(new PositionVoltage(position).withSlot(0));
    });
  }

  public double getPivotPosition(){
    return leader.getPosition().getValueAsDouble();
  }
  
  public Command dropIntake(){
    return run(() -> {
      if(getPivotPosition() < Constants.IntakeConstants.DOWN_POSITION/2){
        requestPosition(Constants.IntakeConstants.DOWN_POSITION);
      }else {
        leader.setControl(new CoastOut());
      }
    });
  }
  
  public Command raiseIntake(){
    return requestPosition(Constants.IntakeConstants.UP_POSITION);
  }
  
  public Command setPivotToCoast(){
    return runOnce(() -> {
      leader.setControl(new CoastOut());
    });
  }

  public Command setRollersToCoast(){
    return runOnce(() -> {
      rollerMotor.setControl(new CoastOut());
    });
  }

  public Command runRollers(){
    return runOnce(() -> {
      rollerMotor.setControl(new VoltageOut(Constants.IntakeConstants.INTAKE_ROLLERS_SPEED));
    });
  }

  public Command stopRollers(){
    return runOnce(() -> {
      rollerMotor.setControl(new CoastOut());
    });
  }

  public Command runRollersReverse(){
    return runOnce(() -> {
      rollerMotor.setControl(new VoltageOut(-Constants.IntakeConstants.INTAKE_ROLLERS_SPEED));
    });
  }

  public Command RunRollersWithVoltage(double voltage){
    return runOnce(() -> {
      rollerMotor.setControl(new VoltageOut(voltage));
    });
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Pivot/Extended", Math.abs(leader.getPosition().getValueAsDouble() - Constants.IntakeConstants.DOWN_POSITION) < 0.02);
    SmartDashboard.putNumber("Intake/Pivot/Position Leader", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Applied Output", leader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Position Follower", follower.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Intake/Rollers/Velocity", rollerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Applied Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Motor Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/Rollers/Running", rollerMotor.getSupplyCurrent().getValueAsDouble() > 0);
  }

}