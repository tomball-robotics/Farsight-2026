package frc.robot.subsystems;

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
import frc.robot.lib.T3Lib.T3Kraken;

public class Shooter extends SubsystemBase {
  
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final CoastOut coastRequest = new CoastOut();
    
  private final SysIdRoutine sysIdRoutine;

  public boolean velocityAdd = false;
  public boolean velocityDec = false;

  public int velocityOffset = 4;

  public Shooter() {
    rightMotor = T3Kraken.createVelocity(
      Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
      NeutralModeValue.Coast,
      true,
      0.0071543, 0.0, 0.0, 0.13
    );
    
    leftMotor = T3Kraken.create(
      Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
      NeutralModeValue.Coast,
      false
    );
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    rightMotor.setControl(coastRequest);

    
    SmartDashboard.putData("Commands/Set Shooter Velocity to Dashboard", setVelocityToDashboard());
    SmartDashboard.putData("Commands/Stop Shooter", stop());
    SmartDashboard.putData("Commands/Set Shooter Velocity to Hub", stationaryVelocityFallback(() -> Odometry.getHubDxDy()[0], () -> Odometry.getHubDxDy()[1]));
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
                   .angularPosition(leftMotor.getPosition().getValue());
              },
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

  public Command stationaryVelocityFallback(Supplier<Double> dxSupplier, Supplier<Double> dySupplier){
    return runOnce(() -> {
      ShotSolution solution = ShotCalculator.solveShot(dxSupplier.get(), dySupplier.get(), 0, 0, 0);
      rightMotor.setControl(new VoltageOut(solution.getVelocity()));
    });
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Velocity", rightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Supply Current", rightMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
  }
}
