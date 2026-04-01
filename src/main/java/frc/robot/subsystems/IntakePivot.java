package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.T3Lib.T3Blink;
import frc.robot.lib.T3Lib.T3Kraken;

public class IntakePivot extends SubsystemBase {

  private final TalonFX leader;
  private final TalonFX follower;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final CoastOut coastRequest = new CoastOut();

  public IntakePivot() {
    leader = T3Kraken.createTalonFXPosition(
      Constants.IntakeConstants.INTAKE_PIVOT_LEADER_ID,
      NeutralModeValue.Coast,
      true,

      50, 0.001, 0.0, 
      20, 0.001, 0.0,
      Constants.IntakeConstants.INTAKE_ENCODER_ID
    );
    

    follower = T3Kraken.create(
      Constants.IntakeConstants.INTAKE_PIVOT_FOLLOWER_ID,
      NeutralModeValue.Coast,
      false
    );

    follower.setControl(new Follower(Constants.IntakeConstants.INTAKE_PIVOT_LEADER_ID, MotorAlignmentValue.Opposed));
    leader.setControl(positionRequest.withPosition(Constants.IntakeConstants.UP_POSITION));

    SmartDashboard.putData("Commands/Drop Intake", dropIntake());
    SmartDashboard.putData("Commands/Raise Intake", raiseIntake());
    SmartDashboard.putData("Commands/Set Pivot to Coast", setPivotToCoast());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", 0);
  }



  public Command requestPosition(double position) {
    return this.run(() -> {
      leader.setControl(positionRequest.withPosition(position));
      SmartDashboard.putNumber("Intake/Pivot/Setpoint", position);
    }).until(() -> Math.abs(leader.getPosition().getValueAsDouble() - position) < 0.02)
      .andThen(new InstantCommand(() -> T3Blink.setFor(.25, T3Blink.Pattern.COLOR2_STROBE)));
  }

  public Command dropIntake() {
    return requestPosition(Constants.IntakeConstants.DOWN_POSITION)
      .andThen(setPivotToCoast());
  }

  public Command raiseIntake() {
    return requestPosition(Constants.IntakeConstants.UP_POSITION);
  }

  public Command slowRaise() {
    return this.run(() -> {
      leader.setControl(positionRequest.withPosition(Constants.IntakeConstants.UP_POSITION).withSlot(1));
      SmartDashboard.putNumber("Intake/Pivot/Setpoint", Constants.IntakeConstants.UP_POSITION);
    }).until(() -> Math.abs(leader.getPosition().getValueAsDouble() - Constants.IntakeConstants.UP_POSITION) < 0.02)
      .andThen(new InstantCommand(() -> T3Blink.setFor(.25, T3Blink.Pattern.COLOR2_STROBE)));
  }
  
  public Command setPivotToCoast() {
    return runOnce(() -> leader.setControl(coastRequest));
  }

  public boolean isIntakeDown() {
    return leader.getPosition().getValueAsDouble() > Constants.IntakeConstants.DOWN_POSITION / 2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Pivot/Extended", isIntakeDown());
    SmartDashboard.putBoolean("Intake/Pivot/Coast", leader.getAppliedControl() instanceof CoastOut);
    SmartDashboard.putNumber("Intake/Pivot/Position Leader", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Position Follower", follower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Stator Current", leader.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Supply Current", leader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Voltage", leader.getMotorVoltage().getValueAsDouble());
  }
}