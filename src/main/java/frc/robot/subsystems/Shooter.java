package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import frc.robot.lib.T3Lib.T3Kraken;



public class Shooter extends SubsystemBase {
  
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final CoastOut coastRequest = new CoastOut();
    
  private final SysIdRoutine sysIdRoutine;

  public boolean velocityAdd = false;
  public boolean velocityDec = false;

  public int velocityOffset = 1;

  public boolean atSetpoint = false;
  double velocitySetpoint = 0;

  private InterpolatingList map = new InterpolatingList();

  public Shooter() {
    rightMotor = T3Kraken.createVelocity(
      Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
      NeutralModeValue.Coast,
      true,
      0.1, 0.0, 0.0, 0.125
    );
    
    leftMotor = T3Kraken.create(
      Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
      NeutralModeValue.Coast,
      false
    );
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    rightMotor.setControl(coastRequest);

    
    SmartDashboard.putData("Commands/Stop Shooter", stop());
    SmartDashboard.putNumber("Shooter/Velocity Manual Set", 0);
    SmartDashboard.putNumber("Shooter/KV Manual Set", 0);


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
                   .angularPosition(leftMotor.getPosition().getValue());
              },
        this
      )
    );
  }
  
  public void setVelocity(double velocity) {  
    rightMotor.setControl(velocityRequest.withVelocity(velocity + (velocityAdd ? velocityOffset : 0) + (velocityDec ? -velocityOffset : 0)));
    velocitySetpoint = velocity + (velocityAdd ? velocityOffset : 0) + (velocityDec ? -velocityOffset : 0);
    SmartDashboard.putNumber("Shooter/Setpoint", velocity); 
  }
  
  public Command setVelocityToDashboard() {
    return runOnce(() -> {
      double velocity = SmartDashboard.getNumber("Shooter/Velocity Manual Set", 0);
      setVelocity(velocity);
      SmartDashboard.putNumber("Shooter/Setpoint", velocity);
    });
  }

  public Command setKV(){
    return runOnce(() -> {
      Slot0Configs con = new Slot0Configs();
      con.kP = 0.1;
      con.kV = SmartDashboard.getNumber("Shooter/KV Manual Set", 0);
      rightMotor.getConfigurator().apply(con);
    });
  }
  
  public Command stop() {
    return runOnce(() -> {
      rightMotor.setControl(coastRequest);
      SmartDashboard.putNumber("Shooter/Setpoint", 0);
    });
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

  public Command shootToHub(Supplier<Double> distance){
    return runOnce(() -> {
      SmartDashboard.putNumber("Measured Distance", distance.get());
      setVelocity(map.get(distance.get()).getVelocity());
    });
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Velocity", rightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Supply Current", rightMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
;
    atSetpoint = Math.abs(velocitySetpoint - rightMotor.getVelocity().getValueAsDouble()) < 2.5;

    SmartDashboard.putBoolean("Shooter/At velocity setpoint", atSetpoint);
    SmartDashboard.putBoolean("Shooter/Velocity Increase" , velocityAdd);
    SmartDashboard.putBoolean("Shooter/Velocity Decrease", velocityDec);
  }
}


/*
 * 2.32 30
 * 2.65 33
 * 3.08 34.5
 * 3.31 35
 * 3.83 37
 * 
 */

class InterpolatingList{
    ArrayList<ShotSolution> tunedSolutions;

    public InterpolatingList(){
        tunedSolutions = new ArrayList<ShotSolution>();
        addSolution(1.83, 31.5, 1);
        addSolution(2.32, 31 + 1.5, 1);
        addSolution(2.65, 33+1.5, 1);
        addSolution(3.08, 35.25+1.5, 1);
        addSolution(3.38+1.5, 37, 1);
        addSolution(3.83+1.5, 38.25, 1);

        /* 
        addSolution(1.77, 31, 1);
        addSolution(2.21, 32.5, 1);
        addSolution(2.42,33.5,1);
        addSolution(2.72, 35, 1);
        addSolution(3.13, 38, 1);
        */

    }

    public void addSolution(double distance, double velocity, double time){
        int index = 0;
        while(index < tunedSolutions.size() && tunedSolutions.get(index).getDistance() < distance){index++;}
        tunedSolutions.add(index, new ShotSolution(time, distance, velocity));
        SmartDashboard.putString("MAP", tunedSolutions.toString());
    }

    public ShotSolution get(double distance){
        if (distance <= tunedSolutions.get(0).getDistance()) return tunedSolutions.get(0);
        if (distance >= tunedSolutions.get(tunedSolutions.size() - 1).getDistance()) return tunedSolutions.get(tunedSolutions.size() - 1);
    
        for (int i = 0; i < tunedSolutions.size() - 1; i++) {
            ShotSolution lo = tunedSolutions.get(i);
            ShotSolution hi = tunedSolutions.get(i + 1);
            if (distance >= lo.getDistance() && distance <= hi.getDistance()) {
                double velocity = interpolate(lo.getDistance(), hi.getDistance(), lo.getVelocity(), hi.getVelocity(), distance);
                double time = interpolate(lo.getDistance(), hi.getDistance(), lo.getTimeOfFlight(), hi.getTimeOfFlight(), distance);
                return new ShotSolution(time, distance, velocity);
            }
        }
        return tunedSolutions.get(tunedSolutions.size() - 1);
    }

    private double interpolate(double x1, double x2, double y1, double y2, double x) {
        if (x1 == x2) {return y1;}
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}