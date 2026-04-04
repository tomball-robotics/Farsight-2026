package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
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
  private final CoastOut coastRequest = new CoastOut();
  
  public Feeder() {
    motor = T3Kraken.create(
      Constants.FeederConstants.FEEDER_MOTOR_ID,
      NeutralModeValue.Coast,
      false
    );

    SmartDashboard.putData("Commands/Run Feeder", run());
    SmartDashboard.putData("Commands/Run Reverse Feeder", runReverse());
    SmartDashboard.putData("Commands/Stop Feeder", stop());
  }
  
  public Command run() {
    return runOnce(() -> motor.setControl(runRequest));
  }
  
  public Command runReverse() {
    return runOnce(() -> motor.setControl(reverseRunRequest));
  }
  
  public Command stop() {
    return runOnce(() -> motor.setControl(coastRequest));
  }
  
  @Override
  public void periodic() {
    double velocity = motor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("Feeder/Velocity", velocity);
    SmartDashboard.putNumber("Feeder/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Feeder/Running", velocity > 0.5);
    
  }
}