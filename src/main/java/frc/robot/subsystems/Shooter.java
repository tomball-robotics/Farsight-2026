// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    new SysIdRoutine.Config(), //May have to customize it
    new SysIdRoutine.Mechanism(
        output -> {velocityMotor.setControl(new VoltageOut(output));},
        null,
        this
    )
);


  public Shooter(){

    TalonFXConfiguration AngleConfig = new TalonFXConfiguration();
    TalonFXConfiguration VelocityConfig = new TalonFXConfiguration();

    AngleConfig.Slot0.kP = 0.5;
    AngleConfig.Slot0.kI = 0;
    AngleConfig.Slot0.kD = 0.001;

    VelocityConfig.Slot0.kP = 0.3;
    VelocityConfig.Slot0.kI = 0;
    VelocityConfig.Slot0.kD = 0;
    VelocityConfig.Slot0.kV = 0.005;

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

    //Collections.sort(distanceSolutions, (d1, d2) -> (Double.compare(d1.distance, d2.distance)));

    
  }

  public Command setAngleAndVelocity(double degrees, double velocity){
    return runOnce (() -> {
      System.out.println("setAngleAndVelocity");

      angleMotor.setControl(request.withPosition(degrees));
      velocityMotor.setControl(new VelocityVoltage(velocity).withSlot(0));

    });
  }

  public Command stop(){
    return runOnce(() -> velocityMotor.setControl(new VoltageOut(0)));
  }

  public DistanceSolution solveForPosition(double distance){
    
    int left = 0;
    int right = distanceSolutions.size() -1;
    while(left < distanceSolutions.size() && distance <= distanceSolutions.get(left+1).distance){
      left++;
    }
    while(right >= 0 && distance >= distanceSolutions.get(right-1).distance){
      right--;
    }
    if(left == right){return distanceSolutions.get(left);}


    double angle = interpolate(distanceSolutions.get(left).distance, distanceSolutions.get(right).distance, distanceSolutions.get(left).angle, distanceSolutions.get(right).angle, distance);
    double velocity = interpolate(distanceSolutions.get(left).distance, distanceSolutions.get(right).distance, distanceSolutions.get(left).velocity, distanceSolutions.get(right).velocity, distance);
    return new DistanceSolution(distance, angle, velocity);
  }

  public double interpolate(double x1, double x2, double y1, double y2, double x3){
    if(x1==x2){return y1;}
    return y1 + (x3 - x1) * (y2 - y1)/(x2-x1);
  }

  public Command aimForHub(Supplier<Double> distance){
    return run(() -> {
      DistanceSolution target = solveForPosition(distance.get());
      angleMotor.setControl(request.withPosition(target.angle));
      velocityMotor.setControl(new VelocityVoltage(target.velocity).withSlot(0));
    });
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/At Velocity", Math.abs(targetVelocityMPS - velocityMotor.getVelocity().getValueAsDouble()) < 0.1);
    SmartDashboard.putBoolean("Shooter/At Angle", Math.abs(targetAngleDegrees - angleMotor.getPosition().getValueAsDouble()) < 0.1);


    SmartDashboard.putNumber("Shooter/Shooter velocity", velocityMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Shooter angle", angleMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Shooter/output voltage", angleMotor.getSupplyCurrent().getValueAsDouble());
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