package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final Field2d field = new Field2d();

    private PIDController xController, yController, yawController;


    public SwerveDrivePoseEstimator m_poseEstimator;

     private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
      

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    //private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    //private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,
        state -> {
            SignalLogger.writeString("state", state.toString()); System.out.println(state.toString());
        }
    ),
    new SysIdRoutine.Mechanism(
        output -> setControl(m_translationCharacterization.withVolts(output)),
        null,
        this
    )
);
    



    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    /* 
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */

     /* 
    private final SysIdRoutine m_sysIdRoutineRotation =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Seconds),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString(
                "SysIdRotation_State",
                state.toString()
            )
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(
                    m_rotationCharacterization
                        .withRotationalRate(output.in(Volts))
                );
                SignalLogger.writeDouble(
                    "Rotational_Rate",
                    output.in(Volts)
                );
            },
            null,
            this
        )
    );
    */
    
    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        Shuffleboard.getTab("Drive")
    .add("Field", field)
    .withWidget(BuiltInWidgets.kField)
    .withSize(6, 4)
    .withPosition(0, 0);  
        setUpPIDs();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        Shuffleboard.getTab("Drive")
    .add("Field", field)
    .withWidget(BuiltInWidgets.kField)
    .withSize(6, 4)
    .withPosition(0, 0);
        setUpPIDs();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        Shuffleboard.getTab("Drive")
    .add("Field", field)
    .withWidget(BuiltInWidgets.kField)
    .withSize(6, 4)
    .withPosition(0, 0);
        setUpPIDs();
    }

    private void setUpPIDs(){
        xController = new PIDController(0.85,0,0);
    
        yController = new PIDController(1.5,0,0);
        yawController = new PIDController(4,0,0);

        yawController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        yawController.setTolerance(0.005);

        m_poseEstimator = new SwerveDrivePoseEstimator(
          getKinematics(),
          getState().Pose.getRotation(),
          getState().ModulePositions,
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

          LimelightHelpers.SetIMUMode("limelight-front", 1);
          LimelightHelpers.SetIMUMode("limelight-back", 1);

    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> {
                    //System.out.println("Estimated X: " + m_poseEstimator.getEstimatedPosition().plus(new Transform2d(0, 0, Rotation2d.k180deg)).getX());
                    //System.out.println("Estimated Y: " + m_poseEstimator.getEstimatedPosition().plus(new Transform2d(0, 0, Rotation2d.k180deg)).getY()); 

                    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {System.out.println("X: " + pose.getX()); System.out.println("Y: " + pose.getY());});

                    



                    return m_poseEstimator.getEstimatedPosition();//.plus(new Transform2d(0, 0, Rotation2d.k180deg));
                    },   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(0.1, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(0.0, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        updateOdometry();
        SmartDashboard.putBoolean("FieldAlive", true);

        field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Robot X:", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Robot Y:", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Robot Yaw: ", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("Target X: ", AutoBuilder.getCurrentPose().getX());
        SmartDashboard.putNumber("Target Y: ", AutoBuilder.getCurrentPose().getY());

        //System.out.println(m_poseEstimator.getEstimatedPosition());
    }

    public Command driveTo(Supplier<Pose2d> t) {
        return Commands.defer(() -> {
            Pose2d target = t.get();
        // Calculate the specific target once at the start
        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        yawController.setSetpoint(target.getRotation().getDegrees());
        return this.run(() -> {
            double rotationSpeed = yawController.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(), target.getRotation().getRadians());
            double xVel = xController.calculate(m_poseEstimator.getEstimatedPosition().getX(), target.getX());
            double yVel = yController.calculate(m_poseEstimator.getEstimatedPosition().getY(), target.getY());

            System.out.println(m_poseEstimator.getEstimatedPosition().getRotation().getRadians() + "       " + target.getRotation().getRadians());
            //System.out.println("Y:" + (yVel > 0.01));
            //System.out.println("Yaw:" + rotationSpeed);
            
            SmartDashboard.putNumber("Target Rotation: ", target.getRotation().getRadians());
            SmartDashboard.putNumber("Actual Rotation: ", m_poseEstimator.getEstimatedPosition().getRotation().getRadians());

            this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withRotationalRate(rotationSpeed)
            );
        })
        .until(this::atSetpoint); // The command ends when it's facing the target
    }, Set.of(this)); // Requirement ensures it overrides joystick drive
}

public boolean atSetpoint(){
    return xController.atSetpoint() && yController.atSetpoint() && yawController.atSetpoint();
}
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Command resetHeading(){
        
        //return runOnce(() -> {resetRotation(getState().RawHeading.rotateBy(new Rotation2d(0)));}); 
        return runOnce(() -> {
            var newYaw =
                getState().Pose.getRotation().rotateBy(Rotation2d.k180deg);
            resetRotation(newYaw);
        });
    }

    public void updateOdometry() {
        m_poseEstimator.update(getPigeon2().getRotation2d().plus(Rotation2d.k180deg), getState().ModulePositions);


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,1e9));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {

    
      LimelightHelpers.SetRobotOrientation("limelight-front", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        //LimelightHelpers.SetIMUMode("limelight-front", 4);

      LimelightHelpers.PoseEstimate frontEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

       LimelightHelpers.SetRobotOrientation("limelight-back", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        //LimelightHelpers.SetIMUMode("limelight-back", 4);

       LimelightHelpers.PoseEstimate backEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
      
      if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(frontEstimate.tagCount == 0 && backEstimate.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        LimelightHelpers.PoseEstimate best = null;
        
        if(frontEstimate.tagCount != backEstimate.tagCount){
            best = frontEstimate.tagCount > backEstimate.tagCount ? frontEstimate : backEstimate;
        }
        else if(frontEstimate.avgTagDist != backEstimate.avgTagDist){
            best = frontEstimate.avgTagDist < backEstimate.avgTagDist ? frontEstimate : backEstimate;
        }
        else{
            best = frontEstimate;
        }
        


        m_poseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.6, 0.6, 999999)
        );
        m_poseEstimator.addVisionMeasurement(
            best.pose,
            best.timestampSeconds);
      }
    }
  }

  public Command allignToTrench(){
    return driveTo(()->new Pose2d(m_poseEstimator.getEstimatedPosition().getX(), 7.25, m_poseEstimator.getEstimatedPosition().getRotation()));
  }

  public Command holdAllignmentToTrench(CommandXboxController joystick){
    return this.run(() -> { 
        double xInput = joystick.getLeftY();
        double deadband = 0.2;
        if (Math.abs(xInput) < deadband) {
            xInput = 0;
        }
    this.setControl(new SwerveRequest.FieldCentric()
    .withRotationalDeadband(MaxAngularRate * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withVelocityX(xInput * MaxSpeed)
    .withVelocityY(yController.calculate(m_poseEstimator.getEstimatedPosition().getY(), 7.2))
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate));} );
  }

  public Command pointTowardsHub(CommandXboxController joystick) {
    return this.run(() -> {

        double robotX = m_poseEstimator.getEstimatedPosition().getX();
        double robotY = m_poseEstimator.getEstimatedPosition().getY();

        double dx = 11.75 - robotX;
        double dy = 4.0  - robotY;

        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));

        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
        SmartDashboard.putNumber("Target Angle (deg)", targetAngle.getDegrees());

        this.setControl(
            new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.2)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(joystick.getLeftY() * MaxSpeed)
                .withVelocityY(joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(
                    yawController.calculate(
                        m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                        targetAngle.getRadians()
                    )
                )
        );
    });
}
  }


