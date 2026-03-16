package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib;

public class Shooter extends SubsystemBase {
  
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final CoastOut coastRequest = new CoastOut();
  
  private final ArrayList<DistanceSolution> distanceSolutions;
  
  public Shooter() {
    rightMotor = T3Lib.createTalonFXVelocity(
    Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
    NeutralModeValue.Coast,
    false,
    0.03039, 0.0, 0.0, 0.12664
    );
    
    leftMotor = T3Lib.createTalonFX(
    Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
    NeutralModeValue.Coast,
    false
    );
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    rightMotor.setControl(coastRequest);
    
    distanceSolutions = new ArrayList<>();
    distanceSolutions.add(new DistanceSolution(1.89,  0, -30));
    distanceSolutions.add(new DistanceSolution(2.496, 0, -35));
    distanceSolutions.add(new DistanceSolution(3.48,  0, -40));
    distanceSolutions.add(new DistanceSolution(4.78,  0, -41.61));
    Collections.sort(distanceSolutions, (d1, d2) -> Double.compare(d1.distance, d2.distance));
    
    SmartDashboard.putNumber("Shooter/Velocity Manual Set", 0);
  }
  
  public Command setVelocity(double velocity) {
    return runOnce(() -> {
      rightMotor.setControl(velocityRequest.withVelocity(velocity));
      SmartDashboard.putNumber("Shooter/Setpoint", velocity);
    });
  }
  
  public Command setVelocityToDashboard() {
    return runOnce(() -> {
      double velocity = SmartDashboard.getNumber("Shooter/Velocity Manual Set", 0);
      rightMotor.setControl(velocityRequest.withVelocity(velocity));
      SmartDashboard.putNumber("Shooter/Setpoint", velocity);
    });
  }
  
  public Command stop() {
    return runOnce(() -> {
      rightMotor.setControl(coastRequest);
      SmartDashboard.putNumber("Shooter/Setpoint", 0);
    });
  }
  
  public Command setToHubVelocity(Supplier<Double> distance) {
    return runOnce(() -> {
      DistanceSolution target = solveForPosition(distance.get());
      rightMotor.setControl(velocityRequest.withVelocity(target.velocity));
      SmartDashboard.putNumber("Shooter/Setpoint", target.velocity);
    });
  }
  
  public DistanceSolution solveForPosition(double distance) {
    if (distance <= distanceSolutions.get(0).distance) return distanceSolutions.get(0);
    if (distance >= distanceSolutions.get(distanceSolutions.size() - 1).distance) return distanceSolutions.get(distanceSolutions.size() - 1);
    
    for (int i = 0; i < distanceSolutions.size() - 1; i++) {
      DistanceSolution lo = distanceSolutions.get(i);
      DistanceSolution hi = distanceSolutions.get(i + 1);
      if (distance >= lo.distance && distance <= hi.distance) {
        double angle    = interpolate(lo.distance, hi.distance, lo.angle,    hi.angle,    distance);
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
    SmartDashboard.putNumber("Shooter/Velocity", rightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Supply Current", rightMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
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