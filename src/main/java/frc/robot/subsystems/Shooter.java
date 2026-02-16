// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.01;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotor.getConfigurator().apply(config);
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    velocityMotor.getConfigurator().apply(config);
  }

  public Command setVelocity(double mps){
    return runOnce(() -> velocityMotor.setControl(new VoltageOut(mps * Constants.ShooterConstants.VOLTS_TO_MPS)));
  }

  public Command setAngle(double degrees){
    return runOnce(() -> angleMotor.setControl(request.withPosition(degrees * Constants.ShooterConstants.TICKS_TO_DEGREES)));
  }

  public double[] solveForPosition(double distance){
    double[] out = new double[2];

    out[0] = interpolate(Math.floor(distance*10)/10.0, Math.ceil(distance*10)/10.0, distanceSolutions[(int)Math.floor(distance * 10)][0], distanceSolutions[(int)Math.ceil(distance*10)][0], distance);
    out[1] = interpolate(Math.floor(distance*10)/10.0, Math.ceil(distance*10)/10.0, distanceSolutions[(int)Math.floor(distance * 10)][1], distanceSolutions[(int)Math.ceil(distance*10)][1], distance);
    return out;
  }

  public double interpolate(double x1, double x2, double y1, double y2, double x3){
    if(x1==x2){return y1;}
    return y1 + (x3 - x1) * (y2 - y1)/(x2-x1);
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
  
}
