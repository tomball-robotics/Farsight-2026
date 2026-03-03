// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


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

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();


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

    driver.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


    //Swerve Controls
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftY() * MaxSpeed + 0.25 * operator.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftX() * MaxSpeed + 0.25 * operator.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (-driver.getRightX() * MaxAngularRate - 0.25 * operator.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

    //driver.leftBumper().onTrue(drivetrain.toggleSlowMode()); //Slowmode
    //driver.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake)); //X-lock

    // driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    //driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    driver.leftTrigger().onTrue(shooter.setAngleAndVelocity(0,-50));
    driver.leftTrigger().onFalse(shooter.setAngleAndVelocity(0, 0));

    driver.rightTrigger().onTrue(new ParallelCommandGroup(indexer.run(1), treadmill.run(1)));
    driver.rightTrigger().onFalse(new ParallelCommandGroup(indexer.stop(), treadmill.stop()));

    driver.rightBumper().onTrue(new ParallelCommandGroup(intakeRollers.run(-1), treadmill.run(1)));
    driver.rightBumper().onFalse(new ParallelCommandGroup(intakeRollers.stop(), Commands.either(Commands.none(), treadmill.stop(), driver.rightTrigger())));

    driver.leftBumper().onTrue(new ParallelCommandGroup(intakeRollers.run(1), treadmill.run(-1), indexer.run(1)));
    driver.leftBumper().onFalse(new ParallelCommandGroup(intakeRollers.stop(), treadmill.stop(), indexer.stop()));
    //driver.rightBumper().onFalse(shooter.stop());
    //driver.leftTrigger().whileTrue(new ParallelCommandGroup(drivetrain.holdAllignmentToTrench(driver), shooter.putDown())); //Allign to nearest trench

    driver.povRight().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.getPigeon2().setYaw(0);}).andThen(drivetrain.resetHeading())); //Reset field view, DONT USE OFTEN
    

    /*Driver Controls*/

    //Intake
    //driver.leftTrigger().onTrue(intakePivot.ifNotDownPutDown().andThen(new ParallelCommandGroup(intakeRollers.run(1), treadmill.run(1)))); //Feed balls to shoot
    // driver.leftTrigger().onTrue(new ParallelCommandGroup(intakeRollers.run(-1), treadmill.run(1)));
    // driver.leftTrigger().onFalse(new ParallelCommandGroup(intakeRollers.stop(), treadmill.stop())); //Stop feeding
    


    //Intake

    driver.povDown().onTrue(intakePivot.dropIntake()); //Intake down
    driver.povUp().onTrue(intakePivot.bringUpIntake());

    driver.a().whileTrue(drivetrain.pointTowardsHub(driver));
    // driver.povUp().onTrue(intakePivot.bringUpIntake()); //Intake up

    /*Operator Controls*/

    //Run Indexer --> Shoot ball
    //driver.rightTrigger().whileTrue(indexer.run(1));
    //driver.rightTrigger().onFalse(indexer.stop());
    

    //Climber Up
    //operator.leftBumper().onTrue(climber.climberUp());
    //operator.leftBumper().onFalse(climber.stop());
    
    //Climber Down
    //operator.rightBumper().onTrue(climber.climberDown()); 
    //operator.rightBumper().onFalse(climber.stop());

    //Shooter
    operator.leftTrigger().whileTrue(new ParallelCommandGroup(drivetrain.pointTowardsHub(driver), shooter.aimForHub(() -> drivetrain.distanceToHub())));
  }
   
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
