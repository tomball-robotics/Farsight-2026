package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib;

public class Rollers extends SubsystemBase {

  TalonFX motor;
  
  public Rollers() {

    motor = new TalonFX(Constants.RollerConstants.ROLLER_MOTOR_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    T3Lib.applyConfig(motor, config);
  }

  public Command run() {
    return runOnce(() -> {
      motor.setControl(new VoltageOut(Constants.RollerConstants.ROLLER_SPEED));
    });
  }

  public Command runReverse() {
    return runOnce(() -> {
      motor.setControl(new VoltageOut(-Constants.RollerConstants.ROLLER_SPEED));
    });
  }

  public Command runWithVoltage(double voltage) {
    return runOnce(() -> {
      motor.setControl(new VoltageOut(voltage));
    });
  }

  public Command stop(){
    return runOnce(() -> {
      motor.setControl(new NeutralOut());
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rollers/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Rollers/Applied Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Rollers/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Rollers/Running", motor.getSupplyCurrent().getValueAsDouble() > 0);
  }
}
