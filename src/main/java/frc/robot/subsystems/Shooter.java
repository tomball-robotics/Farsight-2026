// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.T3Lib;

public class Shooter extends SubsystemBase {
  
  private final TalonFX velocityMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_VELOCITY_MOTOR_ID);
  
  private final ArrayList<DistanceSolution> distanceSolutions;
    
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(
  null, null, null,
  state -> SignalLogger.writeString("state", state.toString())
  ),
  new SysIdRoutine.Mechanism(
  output -> velocityMotor.setControl(new VoltageOut(output)),
  log -> log.motor("TalonFX-" + velocityMotor.getDeviceID())
  .voltage(Volts.of(velocityMotor.getMotorVoltage().getValueAsDouble()))
  .angularVelocity(velocityMotor.getVelocity().getValue())
  .angularPosition(velocityMotor.getPosition().getValue()),
  this
  )
  );
  
  public Shooter() {
    
    TalonFXConfiguration velocityConfig = new TalonFXConfiguration();
    velocityConfig.Slot0.kP = 0.03039;
    velocityConfig.Slot0.kV = 0.12664;
    velocityConfig.Slot0.kA = 0.017248;
    velocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    velocityConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    T3Lib.applyConfig(velocityMotor, velocityConfig);
    
    distanceSolutions = new ArrayList<>();
    distanceSolutions.add(new DistanceSolution(1.89, 0, -30));
    distanceSolutions.add(new DistanceSolution(2.496, 0, -35));
    distanceSolutions.add(new DistanceSolution(3.48, 0, -40));
    distanceSolutions.add(new DistanceSolution(4.78, 0, -41.61));
    Collections.sort(distanceSolutions, (d1, d2) -> Double.compare(d1.distance, d2.distance));
    
    SmartDashboard.putNumber("Shooter/Velocity Manual Set", 0);
  }
  
  public Command setVelocity(double degrees, double velocity) {
    return runOnce(() -> {
      velocityMotor.setControl(new VelocityVoltage(velocity).withSlot(0));
      SmartDashboard.putNumber("Shooter/Setpoint", velocity);
    });
  }

  public Command setVelocityToDashboard() {
    return runOnce(() -> {
      double velocity = SmartDashboard.getNumber("Shooter/Velocity Manual Set", 0);
      velocityMotor.setControl(new VelocityVoltage(velocity).withSlot(0));
      SmartDashboard.putNumber("Shooter/Setpoint", velocity);
    });
  }
  
  public Command stop() {
    return runOnce(() -> {
      velocityMotor.setControl(new CoastOut());
      SmartDashboard.putNumber("Shooter/Setpoint", 0);
    });
  }
  
  public Command aimForHub(Supplier<Double> distance) {
    return run(() -> {
      DistanceSolution target = solveForPosition(distance.get());
      velocityMotor.setControl(new VelocityVoltage(target.velocity).withSlot(0));
      SmartDashboard.putNumber("Shooter/Setpoint", target.velocity);
    });
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
  
  public DistanceSolution solveForPosition(double distance) {
    if (distance <= distanceSolutions.get(0).distance) return distanceSolutions.get(0);
    if (distance >= distanceSolutions.get(distanceSolutions.size() - 1).distance) return distanceSolutions.get(distanceSolutions.size() - 1);
    
    for (int i = 0; i < distanceSolutions.size() - 1; i++) {
      DistanceSolution lo = distanceSolutions.get(i);
      DistanceSolution hi = distanceSolutions.get(i + 1);
      if (distance >= lo.distance && distance <= hi.distance) {
        double angle = interpolate(lo.distance, hi.distance, lo.angle, hi.angle, distance);
        double velocity = interpolate(lo.distance, hi.distance, lo.velocity, hi.velocity, distance);
        return new DistanceSolution(distance, angle, velocity);
      }
    }
    return distanceSolutions.get(distanceSolutions.size() - 1);
  }
  
  private double interpolate(double x1, double x2, double y1, double y2, double x) {
    if (x1 == x2) return y1;
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Velocity", velocityMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Applied Current", velocityMotor.getSupplyCurrent().getValueAsDouble());
  }
}

class DistanceSolution {
  final double distance;
  final double angle;
  final double velocity;
  
  public DistanceSolution(double distance, double angle, double velocity) {
    this.distance = distance;
    this.angle = angle;
    this.velocity = velocity;
  }
}