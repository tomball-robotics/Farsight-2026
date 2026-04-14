package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControlConstants;
import frc.robot.lib.TunerConstants;
import frc.robot.lib.T3Lib.T3Blink;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Odometry;

@SuppressWarnings("unused")
public class RobotContainer {
  
  // swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6; // 80% of kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 1; // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * Constants.ControlConstants.DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.ControlConstants.DEADBAND); // Add a 7% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  
  // subsystems
  private T3Blink blink = new T3Blink();
  private Shooter shooter = new Shooter();
  private IntakePivot intakePivot = new IntakePivot();
  private IntakeRollers intakeRollers = new IntakeRollers();
  private Rollers rollers = new Rollers();
  private Feeder feeder = new Feeder();
  private Swerve drivetrain = TunerConstants.createDrivetrain();
  private Odometry odometry = new Odometry(drivetrain);
  
  // controllers
  private final CommandXboxController driver = new CommandXboxController(ControlConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(ControlConstants.OPERATOR_CONTROLLER_ID);

  // autonomous
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    drivetrain.setOdometry(odometry);
    NamedCommands.registerCommand("Run Shooter", shooter.shootToHub(() -> odometry.distanceToHub()).withTimeout(1.0));
    //NamedCommands.registerCommand("Run Shooter", Commands.runOnce(() -> shooter.setVelocity(33.5)));
    NamedCommands.registerCommand("Stop Shooter", shooter.stop());
    
    NamedCommands.registerCommand("Feed", new ParallelCommandGroup(feeder.runVelocity(), rollers.run()));
    NamedCommands.registerCommand("Stop Feed", new ParallelCommandGroup(feeder.stop(), rollers.stop()));
    
    NamedCommands.registerCommand("Drop Intake", intakePivot.dropIntake());
    NamedCommands.registerCommand("Raise Intake", intakePivot.raiseIntake());
    NamedCommands.registerCommand("Slow Raise Intake", new ParallelCommandGroup(intakePivot.slowRaise(), intakeRollers.run()));
    
    NamedCommands.registerCommand("Run Intake", intakeRollers.run());
    NamedCommands.registerCommand("Stop Intake", intakeRollers.stop());
    
    NamedCommands.registerCommand("Aim", drivetrain.autoPointTowardsHub().withTimeout(0.2));
    NamedCommands.registerCommand("Default", drivetrain.getDefaultCommand());
    NamedCommands.registerCommand("Slow Raise Intake", intakePivot.slowRaise());
    
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings() {

    //Swerve Controls
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftY() * -MaxSpeed)) // Drive forward with negative Y (forward)
        .withVelocityY((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (driver.getLeftX() * -MaxSpeed)) // Drive left with negative X (left)
        .withRotationalRate((drivetrain.slowModeEnabled ? 0.25 : 1.0) * (-driver.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
      )
    );
    
    /* --- driver controls --- */
    
    // brake mode with x
    driver.x().onTrue(drivetrain.applyRequest(() -> brake));
    driver.x().onFalse(drivetrain.getDefaultCommand());

    // hub alignment with left trigger
    driver.leftTrigger().onTrue(drivetrain.pointTowardsHub(driver));
    driver.leftTrigger().onFalse(drivetrain.getDefaultCommand());

    //Allign backwards for funneling
    driver.rightTrigger().onTrue(drivetrain.pointTowardsAngle(driver, Rotation2d.kZero.plus(drivetrain.addedRotation)));
    driver.rightTrigger().onFalse(drivetrain.getDefaultCommand());

    //Shooter testing
    driver.y().onTrue(shooter.setVelocityToDashboard());
    driver.y().onFalse(shooter.stop());

    driver.leftBumper().onTrue(feeder.setKV());
    
    // reset heading with pov right
    driver.povRight().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.getPigeon2().setYaw(0);}).andThen(drivetrain.resetHeading()));

    /* --- operator controls ---  */
    
    // run intake rollers with left bumper
    operator.leftBumper().onTrue(intakeRollers.run());
    operator.leftBumper().onFalse(intakeRollers.stop());
    
    // raise/drop intake with vertical dpad
    operator.povDown().onTrue(intakePivot.dropIntake());
    operator.povUp().onTrue(intakePivot.raiseIntake());
    
    // run shooter
    operator.leftTrigger().onTrue(shooter.shootToHub(() -> odometry.distanceToHub()));
    operator.leftTrigger().onFalse(shooter.stop());
    
    // run feeder & rollers with right trigger
    operator.rightTrigger().onTrue(Commands.waitUntil(() -> shooter.atSetpoint).andThen(new ParallelCommandGroup(feeder.runVelocity(), rollers.run())));
    operator.rightTrigger().onFalse(new ParallelCommandGroup(feeder.stop(), rollers.stop()));

    operator.rightBumper().onTrue(Commands.runOnce(() -> shooter.setVelocity(43)));
    operator.rightBumper().onFalse(shooter.stop());
    
    // twerk with b <---  PINEAPPLE
    operator.b().onTrue(new ParallelCommandGroup(intakePivot.slowRaise(), intakeRollers.run()));
    operator.b().onFalse(intakeRollers.stop());

    
    // reverse feeder with a
    operator.a().onTrue(new ParallelCommandGroup(feeder.runReverse(), rollers.runReverse()));
    operator.a().onFalse(new ParallelCommandGroup(feeder.stop(), rollers.stop()));

    //Velocity Offset
    operator.y().onTrue(shooter.toggleVelocityIncrease());
    operator.x().onTrue(shooter.toggleVelocityDecrease());
    
    /* --- Sys ID Controls */
    
    /* 
    driver.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
    driver.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    driver.y().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    driver.x().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

    driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    */


  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetOrientation(){
    drivetrain.runOnce(() -> {
      drivetrain.seedFieldCentric(); 
      drivetrain.getPigeon2().setYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble() + drivetrain.addedRotation.getDegrees());
    }).andThen(drivetrain.resetHeading()).schedule();
  }
  
}