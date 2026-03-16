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

public class IntakeRollers extends SubsystemBase {

  private final TalonFX motor;

  private final VoltageOut runRequest = new VoltageOut(Constants.IntakeConstants.INTAKE_ROLLERS_SPEED);
  private final VoltageOut reverseRunRequest = new VoltageOut(-Constants.IntakeConstants.INTAKE_ROLLERS_SPEED);
  private final CoastOut coastRequest = new CoastOut();

  public IntakeRollers() {
    motor = T3Lib.createTalonFX(
      Constants.IntakeConstants.INTAKE_ROLLERS_ID,
      NeutralModeValue.Coast,
      false
    );

    motor.setControl(coastRequest);
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
    SmartDashboard.putNumber("Intake/Rollers/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/Rollers/Running", Math.abs(motor.getVelocity().getValueAsDouble()) > 0.5);
  }
}