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
import com.ctre.phoenix6.controls.PositionVoltage;
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
  TalonFX velocityMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_VELOCITY_MOTOR_ID);
  TalonFX angleMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_ANGLE_MOTOR_ID);

  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  
  ArrayList<DistanceSolution> distanceSolutions;

  private double targetAngleDegrees = 0.0;
  private double targetVelocityMPS = 0.0;

  private final SysIdRoutine routine = new SysIdRoutine(
       new SysIdRoutine.Config(
        null,
       null,
        null,
        state -> {
            SignalLogger.writeString("state", state.toString()); System.out.println(state.toString());
        }), //May have to customize it
    new SysIdRoutine.Mechanism(
        output -> {velocityMotor.setControl(new VoltageOut(output));},
        log -> {log.motor("TalonFX-" + velocityMotor.getDeviceID())
           .voltage(Volts.of(velocityMotor.getMotorVoltage().getValueAsDouble()))
           .angularVelocity(velocityMotor.getVelocity().getValue()).angularPosition(velocityMotor.getPosition().getValue());}
          ,
        this
    )
    );


  public Shooter(){

    TalonFXConfiguration AngleConfig = new TalonFXConfiguration();
    TalonFXConfiguration VelocityConfig = new TalonFXConfiguration();

    AngleConfig.Slot0.kP = 1;
    AngleConfig.Slot0.kI = 0.209961;
    AngleConfig.Slot0.kD = 0.001;

    VelocityConfig.Slot0.kP = 0.03039;
    VelocityConfig.Slot0.kI = 0;
    VelocityConfig.Slot0.kD = 0;
    VelocityConfig.Slot0.kV = 0.12664;
    VelocityConfig.Slot0.kA = 0.017248;

    AngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    AngleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    VelocityConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    

    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ShooterConstants.SHOOTER_MIN_ANGLE;
    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


    T3Lib.applyConfig(angleMotor, AngleConfig);
    T3Lib.applyConfig(velocityMotor, VelocityConfig);
    distanceSolutions = new ArrayList<>();
    distanceSolutions.add(new DistanceSolution(1.89, 0, -30));
    distanceSolutions.add(new DistanceSolution(2.496, 0, -35));
    distanceSolutions.add(new DistanceSolution(3.48, 0, -40));
    distanceSolutions.add(new DistanceSolution(4.78, 3, -41.61));
    Collections.sort(distanceSolutions, (d1, d2) -> (Double.compare(d1.distance, d2.distance)));

    angleMotor.setPosition(0);
  }

  public Command setAngleAndVelocity(double degrees, double velocity){
    return runOnce (() -> {
      System.out.println("setAngleAndVelocity");

      targetVelocityMPS = velocity;
      targetAngleDegrees = degrees;

      angleMotor.setControl(request.withPosition(degrees));
      velocityMotor.setControl(new VelocityVoltage(velocity).withSlot(0));

    });
  }

  public Command stop(){
    return runOnce(() -> {velocityMotor.setControl(new CoastOut()); targetVelocityMPS = 0;});
  }

  public DistanceSolution solveForPosition(double distance) {
    if (distance <= distanceSolutions.get(0).distance){return distanceSolutions.get(0);}
    if (distance >= distanceSolutions.get(distanceSolutions.size() - 1).distance){ return distanceSolutions.get(distanceSolutions.size() - 1);}

    for (int i = 0; i < distanceSolutions.size() - 1; i++) {
        if (distance >= distanceSolutions.get(i).distance && distance <= distanceSolutions.get(i + 1).distance) {
            double angle = interpolate(distanceSolutions.get(i).distance, distanceSolutions.get(i+1).distance, distanceSolutions.get(i).angle,    distanceSolutions.get(i+1).angle, distance);
            double velocity = interpolate(distanceSolutions.get(i).distance, distanceSolutions.get(i+1).distance, distanceSolutions.get(i).velocity, distanceSolutions.get(i+1).velocity, distance);
            return new DistanceSolution(distance, angle, velocity);
        }
    }
    return distanceSolutions.get(distanceSolutions.size() - 1);
}

  public double interpolate(double x1, double x2, double y1, double y2, double x3){
    if(x1==x2){return y1;}
    return y1 + (x3 - x1) * (y2 - y1)/(x2-x1);
  }

  public Command aimForHub(Supplier<Double> distance){
    return run(() -> {
      DistanceSolution target = solveForPosition(distance.get());

      targetVelocityMPS = target.velocity;
      targetAngleDegrees = target.angle;

      angleMotor.setControl(request.withPosition(target.angle));
      velocityMotor.setControl(new VelocityVoltage(target.velocity).withSlot(0));
      SmartDashboard.putNumber("Shooter Pivot Target", targetAngleDegrees);
    });
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

  public Command putDown(){
    return setAngleAndVelocity(0,0);
  }
    /* 
  public double degreesToRotations(double degrees){
    return Constants.ShooterConstants.ROTATIONS_PER_DEGREE * (degrees - Constants.ShooterConstants.SHOOTER_MAX_ANGLE);
  }
    */

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter At Velocity", Math.abs(velocityMotor.getVelocity().getValueAsDouble()-targetVelocityMPS) < 2);
    SmartDashboard.putBoolean("Shooter At Angle", Math.abs(velocityMotor.getPosition().getValueAsDouble()-targetAngleDegrees) < 0.2);
    SmartDashboard.putNumber("Shooter Pivot Position", angleMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Velocity Error", Math.abs(velocityMotor.getVelocity().getValueAsDouble()-targetVelocityMPS));
    SmartDashboard.putNumber("Target Velocity", targetVelocityMPS);
  }
}



class DistanceSolution{
  double distance;
  double angle;
  double velocity;

  public DistanceSolution(double d, double a, double v){
    distance = d;
    angle = a;
    velocity = v;
  }
}