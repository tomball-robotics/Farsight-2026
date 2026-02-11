// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystem declerations
  Shooter shooter = new Shooter();
  Climber climber = new Climber();
  IntakePivot intakePivot = new IntakePivot();
  Roller intakeRollers = new Roller(Constants.IntakeRollerConstants.INTAKE_ROLLERS_ID, Constants.IntakeRollerConstants.INTAKE_ROLLERS_SPEED);
  Roller treadmill = new Roller(Constants.TreadmillConstants.TREADMILL_ID, Constants.TreadmillConstants.TREADMILL_SPEED);
  Roller indexer = new Roller(Constants.IndexerConstants.INDEXER_ID, Constants.IndexerConstants.INDEXER_SPEED);

  //Controllers
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_ID);

  //Autonomous
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driver.leftTrigger().onTrue(intakePivot.ifNotDownPutDown().andThen(new ParallelCommandGroup(intakeRollers.run(1), treadmill.run(1))));
    driver.leftTrigger().onFalse(new ParallelCommandGroup(intakeRollers.stop(), treadmill.stop()));
    driver.povDown().onTrue(intakePivot.dropIntake());

    operator.rightTrigger().whileTrue(indexer.run(1));
    operator.leftBumper().onTrue(climber.climberUp());
    //operator.leftBumper().onFalse();
    operator.rightBumper().onTrue(climber.climberDown());
    //operator.rightBumper().onFalse();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
