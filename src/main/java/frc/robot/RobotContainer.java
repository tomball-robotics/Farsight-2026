package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.ControlConstants;
import frc.robot.lib.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollers;
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

  // swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();

  // subsystems
  private Shooter shooter = new Shooter();
  private Intake intake = new Intake();
  private Rollers rollers = new Rollers();
  private Feeder feeder = new Feeder();
  private Swerve drivetrain = TunerConstants.createDrivetrain();

  // controllers
  private final CommandXboxController driver = new CommandXboxController(ControlConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(ControlConstants.OPERATOR_CONTROLLER_ID);

  // autonomous
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Intake Down", intake.dropIntake());
    SmartDashboard.putData("Intake Up", intake.raiseIntake());
    SmartDashboard.putData("Intake Coast", intake.setPivotToCoast());

    SmartDashboard.putData("Shooter Down", shooter.setVelocity(0, -40));
    SmartDashboard.putData("Shooter Coast",shooter.stop());

    NamedCommands.registerCommand("Run Shooter", shooter.aimForHub(() -> drivetrain.distanceToHub()));
    NamedCommands.registerCommand("Stop Shooter", shooter.setVelocity(0, 0));
    NamedCommands.registerCommand("Run Feeder", new ParallelCommandGroup(feeder.run(), rollers.run(), intake.raiseIntake()));

    NamedCommands.registerCommand("Drop Intake", intake.dropIntake());
    NamedCommands.registerCommand("Run Intake", new ParallelCommandGroup(rollers.run(), rollers.run()));
    NamedCommands.registerCommand("Stop Intake", rollers.stop());
    NamedCommands.registerCommand("Raise Intake", intake.raiseIntake());

    NamedCommands.registerCommand("Aim", drivetrain.pointTowardsHub(driver));
    NamedCommands.registerCommand("Default", drivetrain.getDefaultCommand());
  
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

    /* --- driver controls --- */
    
    // brake mode with x
    driver.x().onTrue(drivetrain.applyRequest(() -> brake));
    driver.x().onFalse(drivetrain.getDefaultCommand());

    // hub alignment with left trigger
    driver.leftTrigger().onTrue(drivetrain.pointTowardsHub(driver));
    driver.leftTrigger().onFalse(drivetrain.getDefaultCommand());

    // reset heading with pov right
    driver.povRight().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.getPigeon2().setYaw(0);}).andThen(drivetrain.resetHeading()));

    /* --- operator controls ---  */

    // run intake rollers with left bumper
    operator.leftBumper().onTrue(intake.runRollers());
    operator.leftBumper().onFalse(intake.stopRollers());

    // raise/drop intake with vertical dpad
    operator.povDown().onTrue(intake.dropIntake());
    operator.povUp().onTrue(intake.raiseIntake());

    // run shooter
    operator.leftTrigger().whileTrue(shooter.aimForHub(() -> drivetrain.distanceToHub()));
    operator.leftTrigger().onFalse(shooter.setVelocity(0, 0));

    // run feeder & rollers with right trigger
    operator.rightTrigger().onTrue(new ParallelCommandGroup(feeder.run(), rollers.run(), intake.runRollers()));
    operator.rightTrigger().onFalse(new ParallelCommandGroup(feeder.stop(), rollers.stop(), intake.stopRollers()));

    // twerk with b
    operator.b().onTrue(intake.raiseIntake());
    operator.b().onFalse(intake.dropIntake());

    // reverse feeder with a
    operator.a().onTrue(feeder.runReverse());
    operator.a().onFalse(feeder.stop());
  }
   
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
