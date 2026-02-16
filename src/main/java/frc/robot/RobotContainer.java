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
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {

    //Driver Controls

    //Intake
    driver.leftTrigger().onTrue(intakePivot.ifNotDownPutDown().andThen(new ParallelCommandGroup(intakeRollers.run(1), treadmill.run(1))));
    driver.leftTrigger().onFalse(new ParallelCommandGroup(intakeRollers.stop(), treadmill.stop()));

    
    
    //Drop Intake
    driver.povDown().onTrue(intakePivot.dropIntake());


    //Operator Controls

    //Run Indexer --> Shoot ball
    operator.rightTrigger().whileTrue(indexer.run(1));
    operator.rightTrigger().onFalse(indexer.stop());
    

    //Climber Up
    operator.leftBumper().onTrue(climber.climberUp());
    operator.leftBumper().onFalse(climber.stop());
    
    //Climber Down
    operator.rightBumper().onTrue(climber.climberDown());
    operator.rightBumper().onFalse(climber.stop());


  }
   
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
