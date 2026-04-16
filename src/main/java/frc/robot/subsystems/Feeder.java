package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib.T3Kraken;

public class Feeder extends SubsystemBase {
  
  private final TalonFX motor;
  
  private final VoltageOut runRequest = new VoltageOut(Constants.FeederConstants.FEEDER_SPEED);
  private final VoltageOut reverseRunRequest = new VoltageOut(-Constants.FeederConstants.FEEDER_SPEED);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(Constants.FeederConstants.FEEDER_VELOCITY);
  private final CoastOut coastRequest = new CoastOut();
  
  public Feeder() {
    motor = T3Kraken.createVelocity(
      Constants.FeederConstants.FEEDER_MOTOR_ID,
      NeutralModeValue.Coast,
      false,
      3,
      0.001,
      0,
      0.105
    );

    SmartDashboard.putData("Commands/Run Feeder", run());
    SmartDashboard.putData("Commands/Run Reverse Feeder", runReverse());
    SmartDashboard.putData("Commands/Stop Feeder", stop());
    SmartDashboard.putNumber("Feeder/KV Manual Set", 0);

  }
  
  public Command run() {
    return runOnce(() -> motor.setControl(runRequest));
  }

  public Command runVelocity(){
    return runOnce(() -> motor.setControl(velocityRequest.withVelocity(Constants.FeederConstants.FEEDER_VELOCITY)));
  }
  
  public Command runReverse() {
    return runOnce(() -> motor.setControl(reverseRunRequest));
  }
  
  public Command stop() {
    return runOnce(() -> motor.setControl(coastRequest));
  }
/* 
    public Command setKV(){
    return runOnce(() -> {
      Slot0Configs con = new Slot0Configs();
      con.kP = 0.1;
      con.kV = SmartDashboard.getNumber("Feeder/KV Manual Set", 0);
      motor.getConfigurator().apply(con);
    });
  }
    */
  
  @Override
  public void periodic() {
    double velocity = motor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("Feeder/Velocity", velocity);
    SmartDashboard.putNumber("Feeder/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Feeder/Running", velocity > 0.5);
    SmartDashboard.putBoolean("Feeder/Feeder at Setpoint", Math.abs(motor.getVelocity().getValueAsDouble() - Constants.FeederConstants.FEEDER_VELOCITY) < 1);
    
  }
}