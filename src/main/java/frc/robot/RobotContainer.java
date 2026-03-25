package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControlConstants;
import frc.robot.commands.SetupShot;
import frc.robot.lib.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Odometry;


public class RobotContainer {
  
  // swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  
  // subsystems
  private Shooter shooter = new Shooter();
  private IntakePivot intakePivot = new IntakePivot();
  private IntakeRollers intakeRollers = new IntakeRollers();
  private Rollers rollers = new Rollers();
  private Feeder feeder = new Feeder();
  private Climber climber = new Climber();
  private Swerve drivetrain = TunerConstants.createDrivetrain();
  private Odometry odometry = new Odometry(drivetrain);

  
  // controllers
  private final CommandXboxController driver = new CommandXboxController(ControlConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(ControlConstants.OPERATOR_CONTROLLER_ID);

  //commands
  private SetupShot setupShot = new SetupShot(drivetrain, shooter, odometry, driver);
  
  // autonomous
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    drivetrain.setOdometry(odometry);
    NamedCommands.registerCommand("Run Shooter", setupShot);
    NamedCommands.registerCommand("Stop Shooter", shooter.stop());
    
    NamedCommands.registerCommand("Feed", new ParallelCommandGroup(feeder.run(), rollers.run()));
    NamedCommands.registerCommand("Stop Feed", new ParallelCommandGroup(feeder.stop(), rollers.stop()));
    
    NamedCommands.registerCommand("Drop Intake", intakePivot.dropIntake());
    NamedCommands.registerCommand("Raise Intake", intakePivot.raiseIntake());
    
    NamedCommands.registerCommand("Run Intake", intakeRollers.run());
    NamedCommands.registerCommand("Stop Intake", intakeRollers.stop());
    
    NamedCommands.registerCommand("Aim", drivetrain.autoPointTowardsHub().withTimeout(1.5));
    NamedCommands.registerCommand("Default", drivetrain.getDefaultCommand());

    NamedCommands.registerCommand("Climbers Up", climber.autoClimberUp());
    NamedCommands.registerCommand("Climbers Down", climber.autoClimberDown());

    NamedCommands.registerCommand("Climber Auto Home", climber.autonomousHome());
    
    
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings() {
    //Swerve Controls
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftY() * MaxSpeed + 0.25 * operator.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
        .withVelocityY((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftX() * MaxSpeed + 0.25 * operator.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
        .withRotationalRate((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (-driver.getRightX() * MaxAngularRate - 0.25 * operator.getRightX() * MaxAngularRate * 2)) // Drive counterclockwise with negative X (left)
      )
    );
    
    /* --- driver controls --- */
    
    // brake mode with x
    //driver.x().onTrue(drivetrain.applyRequest(() -> brake));
    //driver.x().onFalse(drivetrain.getDefaultCommand());

    /* --- Sys ID Controls */

    driver.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
    driver.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    driver.y().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    driver.x().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
    
    // hub alignment with left trigger
    driver.leftTrigger().onTrue(drivetrain.pointTowardsHub(driver));
    driver.leftTrigger().onFalse(drivetrain.getDefaultCommand());
    
    // reset heading with pov right
    driver.povRight().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.getPigeon2().setYaw(0);}).andThen(drivetrain.resetHeading()));
    
    driver.leftBumper().onTrue(climber.climberUp());
    driver.leftBumper().onFalse(climber.stop());

    driver.rightBumper().onTrue(climber.climberDown());
    driver.rightBumper().onFalse(climber.stop());

    /* --- operator controls ---  */
    
    // run intake rollers with left bumper
    operator.leftBumper().onTrue(intakeRollers.run());
    operator.leftBumper().onFalse(intakeRollers.stop());
    
    // raise/drop intake with vertical dpad
    operator.povDown().onTrue(intakePivot.dropIntake());
    operator.povUp().onTrue(intakePivot.raiseIntake());
    
    // run shooter
    operator.leftTrigger().whileTrue(setupShot);
    operator.leftTrigger().onFalse(new ParallelCommandGroup(shooter.stop(), drivetrain.getDefaultCommand()));
    
    // run feeder & rollers with right trigger
    operator.rightTrigger().onTrue(new ParallelCommandGroup(feeder.run(), rollers.run(), intakeRollers.run()));
    operator.rightTrigger().onFalse(new ParallelCommandGroup(feeder.stop(), rollers.stop(), intakeRollers.stop()));
    
    // twerk with b
    operator.b().onTrue(intakePivot.raiseIntake());
    operator.b().onFalse(intakePivot.dropIntake());
    
    // reverse feeder with a
    operator.a().onTrue(feeder.runReverse());
    operator.a().onFalse(feeder.stop());

    //Velocity Offset
    operator.start().onTrue(shooter.toggleVelocityIncrease());
    operator.back().onTrue(shooter.toggleVelocityDecrease());
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
}

//driver.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
//driver.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
//driver.y().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
//driver.x().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
