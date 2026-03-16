package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib;

public class Rollers extends SubsystemBase {
  
  private final TalonFX motor;
  
  private final VoltageOut runRequest = new VoltageOut(Constants.RollerConstants.ROLLER_SPEED);
  private final VoltageOut reverseRunRequest = new VoltageOut(-Constants.RollerConstants.ROLLER_SPEED);
  private final CoastOut coastRequest = new CoastOut();
  
  public Rollers() {
    motor = T3Lib.createTalonFX(
      Constants.RollerConstants.ROLLER_MOTOR_ID,
      NeutralModeValue.Coast,
      false
    );

    SmartDashboard.putData("Commands/Run Rollers", run());
    SmartDashboard.putData("Commands/Run Reverse Rollers", runReverse());
    SmartDashboard.putData("Commands/Stop Rollers", stop());
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

    SmartDashboard.putNumber("Rollers/Velocity", velocity);
    SmartDashboard.putNumber("Rollers/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Rollers/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Rollers/Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Rollers/Running", velocity > 0.5);
  }
}