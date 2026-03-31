package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib.T3Blink;
import frc.robot.lib.T3Lib.T3Kraken;

public class IntakeRollers extends SubsystemBase {
  
  private final TalonFX motor;
  
  private final VoltageOut runRequest = new VoltageOut(Constants.IntakeConstants.INTAKE_ROLLERS_SPEED);
  private final VoltageOut reverseRunRequest = new VoltageOut(-Constants.IntakeConstants.INTAKE_ROLLERS_SPEED);
  private final CoastOut coastRequest = new CoastOut();
  
  public IntakeRollers() {
    motor = T3Kraken.create(
      Constants.IntakeConstants.INTAKE_ROLLER_ID,
      NeutralModeValue.Coast,
      false
    );

    SmartDashboard.putData("Commands/Run Intake Rollers", run());
    SmartDashboard.putData("Commands/Run Reverse Intake Rollers", runReverse());
    SmartDashboard.putData("Commands/Stop Intake Rollers", stop());
  }
  
  public Command run() {
    return runOnce(() -> {
      motor.setControl(runRequest);
      T3Blink.set(T3Blink.Pattern.COLOR_WAVES_COLOR1_AND_COLOR2);
    });
  }
  
  public Command runReverse() {
    return runOnce(() -> motor.setControl(reverseRunRequest));
  }
  
  public Command stop() {
    return runOnce(() -> {
      motor.setControl(coastRequest);
      T3Blink.set(T3Blink.Pattern.BLACK);
    });
  }
  
  @Override
  public void periodic() {
    double velocity = motor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("Intake Rollers/Velocity", velocity);
    SmartDashboard.putNumber("Intake Rollers/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Rollers/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Rollers/Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Intake Rollers/Running", velocity > 0.5);
  }
}