package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.lib.T3Lib;

public class Shooter extends SubsystemBase {
  
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final CoastOut coastRequest = new CoastOut();
  
  private final ArrayList<DistanceSolution> distanceSolutions;
  
  private final SysIdRoutine sysIdRoutine;

  public boolean velocityAdd = false;
  public boolean velocityDec = false;

  public int velocityOffset = 4;

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
    distanceSolutions.add(new DistanceSolution(1.89,  -30));
    distanceSolutions.add(new DistanceSolution(2.496, -35));
    distanceSolutions.add(new DistanceSolution(3.48,  -40));
    distanceSolutions.add(new DistanceSolution(4.78,  -41.61));
    Collections.sort(distanceSolutions, (d1, d2) -> Double.compare(d1.distance, d2.distance));
    
    SmartDashboard.putData("Commands/Set Shooter Velocity to Dashboard", setVelocityToDashboard());
    SmartDashboard.putData("Commands/Set Shooter Velocity to Hub", setToHubVelocity(() -> SmartDashboard.getNumber("Odometry/Distance to Hub", 0)));
    SmartDashboard.putData("Commands/Stop Shooter", stop());
    SmartDashboard.putNumber("Shooter/Velocity Manual Set", 0);

    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
    null, null, null,
        state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        output -> rightMotor.setControl(new VoltageOut(output)),
        log -> {log.motor("TalonFX-" + rightMotor.getDeviceID())
              .voltage(Volts.of(rightMotor.getMotorVoltage().getValueAsDouble()))
              .angularVelocity(rightMotor.getVelocity().getValue())
              .angularPosition(rightMotor.getPosition().getValue());
        log.motor("TalonFX-" + leftMotor.getDeviceID())
              .voltage(Volts.of(leftMotor.getMotorVoltage().getValueAsDouble()))
              .angularVelocity(leftMotor.getVelocity().getValue())
              .angularPosition(leftMotor.getPosition().getValue());},
        this
      )
    );
  }
  
  public void setVelocity(double velocity) {  
    rightMotor.setControl(velocityRequest.withVelocity(velocity + (velocityAdd ? velocityOffset : 0) + (velocityDec ? -velocityOffset : 0)));
    SmartDashboard.putNumber("Shooter/Setpoint", velocity); 
  }
  
  public Command setVelocityToDashboard() {
    return runOnce(() -> {
      double velocity = SmartDashboard.getNumber("Shooter/Velocity Manual Set", 0);
      setVelocity(velocity);
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
      setVelocity(target.velocity);
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
        double velocity = interpolate(lo.distance, hi.distance, lo.velocity, hi.velocity, distance);
        return new DistanceSolution(distance, velocity);
      }
    }
    return distanceSolutions.get(distanceSolutions.size() - 1);
  }
  
  private double interpolate(double x1, double x2, double y1, double y2, double x) {
    if (x1 == x2) return y1;
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command toggleVelocityIncrease(){
    return runOnce(() -> velocityAdd = !velocityAdd);
  }

  public Command toggleVelocityDecrease(){
    return runOnce(() -> velocityDec = !velocityDec);
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
  final double velocity;
  
  public DistanceSolution(double distance, double velocity) {
    this.distance = distance;
    this.velocity = velocity;
  }
}