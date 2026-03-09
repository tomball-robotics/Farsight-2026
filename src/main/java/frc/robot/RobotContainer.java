// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  //Subsystem declerations
  private Shooter shooter = new Shooter();
  //private Climber climber = new Climber();
  private IntakePivot intakePivot = new IntakePivot();
  private Roller intakeRollers = new Roller(Constants.IntakeRollerConstants.INTAKE_ROLLERS_ID, Constants.IntakeRollerConstants.INTAKE_ROLLERS_SPEED);
  private Roller treadmill = new Roller(Constants.TreadmillConstants.TREADMILL_ID, Constants.TreadmillConstants.TREADMILL_SPEED);
  private Roller indexer = new Roller(Constants.IndexerConstants.INDEXER_ID, Constants.IndexerConstants.INDEXER_SPEED);
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();


  //Controllers
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_ID);

  //Autonomous
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Intake Down", intakePivot.dropIntake());
    SmartDashboard.putData("Intake Up", intakePivot.bringUpIntake());
    SmartDashboard.putData("Intake Coast", intakePivot.coast());

    SmartDashboard.putData("Shooter Down", shooter.setAngleAndVelocity(0, -40));
    SmartDashboard.putData("Shooter Coast",shooter.stop());

    NamedCommands.registerCommand("Run Shooter", shooter.aimForHub(() -> drivetrain.distanceToHub()));
    NamedCommands.registerCommand("Stop Shooter", shooter.setAngleAndVelocity(0, 0));
    NamedCommands.registerCommand("Run Feeder", new ParallelCommandGroup(indexer.run(1), treadmill.run(1), intakeRollers.run(-1)));

    NamedCommands.registerCommand("Drop Intake", intakePivot.dropIntake());
    NamedCommands.registerCommand("Run Intake", new ParallelCommandGroup(intakeRollers.run(-1), treadmill.run(1)));
    NamedCommands.registerCommand("Stop Intake", intakeRollers.stop());
    NamedCommands.registerCommand("Raise Intake", intakePivot.bringUpIntake());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    //driver.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
    //driver.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    //driver.y().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    //driver.x().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

    //Swerve Controls
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftY() * MaxSpeed + 0.25 * operator.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftX() * MaxSpeed + 0.25 * operator.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (-driver.getRightX() * MaxAngularRate - 0.25 * operator.getRightX() * MaxAngularRate * 2)) // Drive counterclockwise with negative X (left)
            )
        );
    
    operator.leftBumper().onTrue(new ParallelCommandGroup(intakeRollers.run(-1)));
    operator.leftBumper().onFalse(new ParallelCommandGroup(intakeRollers.stop()));

    driver.rightTrigger().whileTrue(drivetrain.holdAllignmentToTrench(driver));

    driver.rightBumper().onTrue(drivetrain.applyRequest(() -> brake));
    driver.rightBumper().onFalse(drivetrain.getDefaultCommand());

    driver.start().onTrue(drivetrain.pointTowardsHub(driver));
    driver.start().onFalse(drivetrain.getDefaultCommand());

    driver.povRight().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.getPigeon2().setYaw(0);}).andThen(drivetrain.resetHeading())); //Reset field view, DONT USE OFTEN
    operator.povDown().onTrue(intakePivot.dropIntake()); //Intake down
    operator.povUp().onTrue(intakePivot.bringUpIntake());

    operator.b().onTrue(intakePivot.bringUpIntake());
    operator.b().onFalse(intakePivot.dropIntake());

    operator.leftTrigger().whileTrue(shooter.setAngleAndVelocity(0, -45));
    operator.leftTrigger().onFalse(shooter.setAngleAndVelocity(0, 0));

    operator.rightTrigger().onTrue(new ParallelCommandGroup(indexer.run(1), treadmill.run(1), intakeRollers.run(-1)));
    operator.rightTrigger().onFalse(new ParallelCommandGroup(indexer.stop(), treadmill.stop(), intakeRollers.stop()));
    
    //Climber Up
    //operator.leftBumper().onTrue(climber.climberUp());
    //operator.leftBumper().onFalse(climber.stop());
    
    //Climber Down
    //operator.rightBumper().onTrue(climber.climberDown()); 
    //operator.rightBumper().onFalse(climber.stop());

    operator.a().onTrue(new ParallelCommandGroup(indexer.run(-1), treadmill.run(-1)));
    operator.a().onFalse(new ParallelCommandGroup(indexer.stop(), treadmill.stop()));

  }
   
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
