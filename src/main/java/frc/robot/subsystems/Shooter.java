// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
  TalonFX velocityMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_VELOCITY_MOTOR_ID);
  TalonFX angleMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_ANGLE_MOTOR_ID);
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  double[][] distanceSolutions;

  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),//May have to customize it
    new SysIdRoutine.Mechanism(
        output -> {velocityMotor.setControl(new VoltageOut(output));},
        null,
        this
    )
);


  public Shooter() {

    TalonFXConfiguration AngleConfig = new TalonFXConfiguration();
    TalonFXConfiguration VelocityConfig = new TalonFXConfiguration();

    AngleConfig.Slot0.kP = 0.01;
    AngleConfig.Slot0.kI = 0;
    AngleConfig.Slot0.kD = 0;

    VelocityConfig.Slot0.kP = 0.01;
    VelocityConfig.Slot0.kI = 0;
    VelocityConfig.Slot0.kD = 0;
    VelocityConfig.Slot0.kV = 0;

    AngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    AngleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    VelocityConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ShooterConstants.SHOOTER_MAX_ANGLE * Constants.ShooterConstants.TICKS_TO_DEGREES;
    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ShooterConstants.SHOOTER_MIN_ANGLE * Constants.ShooterConstants.TICKS_TO_DEGREES;
    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


    angleMotor.getConfigurator().apply(AngleConfig);
    velocityMotor.getConfigurator().apply(VelocityConfig);

    
  }

  public Command setVelocity(double mps){
    return run(() -> velocityMotor.setControl(new VelocityVoltage(mps * Constants.ShooterConstants.VOLTS_TO_MPS).withSlot(0)));
  }

  public Command setAngle(double degrees){
    return run(() -> angleMotor.setControl(request.withPosition(degrees * Constants.ShooterConstants.TICKS_TO_DEGREES)));
  }

  public double[] solveForPosition(double distance){
    double[] out = new double[2];

    out[0] = clamp(interpolate(Math.floor(distance*10)/10.0, Math.ceil(distance*10)/10.0, distanceSolutions[(int)Math.floor(distance * 10)][0], distanceSolutions[(int)Math.ceil(distance*10)][0], distance), Constants.ShooterConstants.SHOOTER_MIN_ANGLE, Constants.ShooterConstants.SHOOTER_MAX_ANGLE);
    out[1] = interpolate(Math.floor(distance*10)/10.0, Math.ceil(distance*10)/10.0, distanceSolutions[(int)Math.floor(distance * 10)][1], distanceSolutions[(int)Math.ceil(distance*10)][1], distance);
    return out;
  }

  public double interpolate(double x1, double x2, double y1, double y2, double x3){
    if(x1==x2){return y1;}
    return y1 + (x3 - x1) * (y2 - y1)/(x2-x1);
  }

  public Command aimForHub(Supplier<Double> distance){
    return run(() -> {
      double[] target = solveForPosition(distance.get());
      setAngle(target[0]);
      setVelocity(target[1]);
    });
  }


  @Override
  public void periodic() {

  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }
  
  public static double clamp(double value, double min, double max) {
        // Ensures the value is not less than 'min' and not greater than 'max'.
        return Math.max(min, Math.min(max, value));
    }
}
