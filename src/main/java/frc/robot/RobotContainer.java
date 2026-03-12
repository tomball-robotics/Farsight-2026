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

  // swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // subsystems
  private Shooter shooter = new Shooter();
  private IntakePivot intakePivot = new IntakePivot();
  private Roller intakeRollers = new Roller(Constants.IntakeRollerConstants.INTAKE_ROLLERS_ID, Constants.IntakeRollerConstants.INTAKE_ROLLERS_SPEED);
  private Roller treadmill = new Roller(Constants.TreadmillConstants.TREADMILL_ID, Constants.TreadmillConstants.TREADMILL_SPEED);
  private Roller indexer = new Roller(Constants.IndexerConstants.INDEXER_ID, Constants.IndexerConstants.INDEXER_SPEED);
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();


  // controllers
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_ID);

  // autonomous
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Intake Down", intakePivot.dropIntake());
    SmartDashboard.putData("Intake Up", intakePivot.bringUpIntake());
    SmartDashboard.putData("Intake Coast", intakePivot.coast());

    SmartDashboard.putData("Shooter Down", shooter.setVelocity(0, -40));
    SmartDashboard.putData("Shooter Coast",shooter.stop());

    NamedCommands.registerCommand("Run Shooter", shooter.aimForHub(() -> drivetrain.distanceToHub()));
    NamedCommands.registerCommand("Stop Shooter", shooter.setVelocity(0, 0));
    NamedCommands.registerCommand("R  un Feeder", new ParallelCommandGroup(indexer.run(1), treadmill.run(1), intakePivot.bringUpIntake()));

    NamedCommands.registerCommand("Drop Intake", intakePivot.dropIntake());
    NamedCommands.registerCommand("Run Intake", new ParallelCommandGroup(intakeRollers.run(-1), treadmill.run(1)));
    NamedCommands.registerCommand("Stop Intake", intakeRollers.stop());
    NamedCommands.registerCommand("Raise Intake", intakePivot.bringUpIntake());

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
    operator.leftBumper().onTrue(intakeRollers.run(-1));
    operator.leftBumper().onFalse(intakeRollers.stop());

    // raise/drop intake with vertical dpad
    operator.povDown().onTrue(intakePivot.dropIntake());
    operator.povUp().onTrue(intakePivot.bringUpIntake());

    // run shooter (manual)
    operator.leftTrigger().whileTrue(shooter.aimForHub(() -> drivetrain.distanceToHub()));
    operator.leftTrigger().onFalse(shooter.setVelocity(0, 0));

    // run feeder & treadmill with right trigger
    operator.rightTrigger().onTrue(new ParallelCommandGroup(indexer.run(1), treadmill.run(1), intakeRollers.run(-1)));
    operator.rightTrigger().onFalse(new ParallelCommandGroup(indexer.stop(), treadmill.stop(), intakeRollers.stop()));

    // twerk with b
    operator.b().onTrue(intakePivot.bringUpIntake());
    operator.b().onFalse(intakePivot.dropIntake());

    // reverse feeder with a
    operator.a().onTrue(indexer.run(-1));
    operator.a().onFalse(indexer.stop());

  }
   
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
