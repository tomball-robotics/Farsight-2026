// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Scanner;
import java.util.function.Supplier;

import java.io.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
  TalonFX velocityMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_VELOCITY_MOTOR_ID);
  TalonFX angleMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_ANGLE_MOTOR_ID);

  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  
  ArrayList<> distanceSolutions;

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

    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToRotations(Constants.ShooterConstants.SHOOTER_MAX_ANGLE);
    AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToRotations(Constants.ShooterConstants.SHOOTER_MIN_ANGLE);
    AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


    angleMotor.getConfigurator().apply(AngleConfig);
    velocityMotor.getConfigurator().apply(VelocityConfig);

    try{
      File file = new File(Filesystem.getDeployDirectory(), "shootingData.txt");

      Scanner scan = new Scanner(file);
      distanceSolutions = new double[96][2];
      int i = 0;
      while(scan.hasNextLine()){
        String[] raw = scan.nextLine().split(",");
        distanceSolutions[i][0] = Double.parseDouble(raw[0]);
        distanceSolutions[i][1] = Double.parseDouble(raw[1]);

        i++;

      }
      scan.close();

    }catch(Exception e){e.printStackTrace();}


    
  }

  public Command setVelocity(double mps){
    return run(() -> {
      targetVelocityMPS = mps * Constants.ShooterConstants.MPS_TO_ROTATIONS;
      velocityMotor.setControl(new VelocityVoltage(mps * Constants.ShooterConstants.MPS_TO_ROTATIONS).withSlot(0));
      
    });
  }

  public Command setAngle(double degrees){
    return run(() -> {
      targetAngleDegrees = degreesToRotations(degrees);
      angleMotor.setControl(request.withPosition(degreesToRotations(degrees)));
    });
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
      angleMotor.setControl(request.withPosition(degreesToRotations(target[0])));
      velocityMotor.setControl(new VelocityVoltage(target[1] * Constants.ShooterConstants.MPS_TO_ROTATIONS).withSlot(0));
    });
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/At Velocity", Math.abs(targetVelocityMPS - velocityMotor.getVelocity().getValueAsDouble()) < 0.1);
    SmartDashboard.putBoolean("Shooter/At Angle", Math.abs(targetAngleDegrees - angleMotor.getPosition().getValueAsDouble()) < 0.1);


    SmartDashboard.putNumber("Shooter/Shooter velocity", velocityMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Shooter angle", angleMotor.getPosition().getValueAsDouble());
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
    return setAngle(Constants.ShooterConstants.SHOOTER_MAX_ANGLE);
  }

  public double degreesToRotations(double degrees){
    return Constants.ShooterConstants.ROTATIONS_PER_DEGREE * (degrees - Constants.ShooterConstants.SHOOTER_MAX_ANGLE);
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