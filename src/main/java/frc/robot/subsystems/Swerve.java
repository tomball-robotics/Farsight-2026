package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.lib.TunerConstants;
import frc.robot.lib.TunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    
    private static final double kSimLoopPeriod = 0.005;
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg; 
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;

    public Rotation2d addedRotation = DriverStation.getAlliance()
    .map(a -> a == Alliance.Blue ? kBlueAlliancePerspectiveRotation : kRedAlliancePerspectiveRotation)
    .orElse(kBlueAlliancePerspectiveRotation);
    
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(null, Volts.of(4),null,
        state -> SignalLogger.writeString("state", state.toString())), 
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),null,this)
    );
    
    
    public boolean slowModeEnabled = false;
    public Pose2d pose = new Pose2d();
    
    private PIDController xController, yController, yawController;
    private boolean m_hasAppliedOperatorPerspective = false;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    Odometry odometry;
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleLocations());
    
    
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        initialize();
    }
    
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        initialize();
    }
    
    public Swerve(SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    Matrix<N3, N1> odometryStandardDeviation,
    Matrix<N3, N1> visionStandardDeviation,
    SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        initialize();
    }
    
    private void initialize() {
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        setUpPIDs();
    }

    public void setOdometry(Odometry odem){odometry = odem;}
    
    private void setUpPIDs() {
        xController = new PIDController(0.85, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        yawController = new PIDController(7.67, 0.04, 0);
        
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        yawController.setTolerance(0.001);
    }
    
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(1.5, 0, 0),
                    new PIDConstants(1.5, 0, 0)  
                ),
                config,
                () -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red),
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
    }
    
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }
    
    public Command driveTo(Supplier<Pose2d> targetSupplier) {
        return Commands.defer(() -> {
            Pose2d target = targetSupplier.get();
            return this.run(() -> {
                double xVel = xController.calculate(pose.getX(), target.getX());
                double yVel = yController.calculate(pose.getY(), target.getY());
                double rotationSpeed = yawController.calculate(
                pose.getRotation().getRadians(),
                target.getRotation().getRadians()
                );
                
                SmartDashboard.putNumber("Target Rotation", target.getRotation().getRadians());
                SmartDashboard.putNumber("Actual Rotation", pose.getRotation().getRadians());
                
                this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withRotationalRate(rotationSpeed)
                );
            }).until(this::atSetpoint);
        }, Set.of(this));
    }
    
    public boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint() && yawController.atSetpoint();
    }
    
    public Command holdAlignmentToTrench(CommandXboxController joystick) {
        return this.run(() -> {
            double xInput = joystick.getLeftY();
            if (Math.abs(xInput) < Constants.ControlConstants.DEADBAND) xInput = 0;
            
            double midTrenchY = (Constants.SwervePositions.rightTrenchY + Constants.SwervePositions.leftTrenchY) / 2;
            double targetY = pose.getY() > midTrenchY ? Constants.SwervePositions.rightTrenchY : Constants.SwervePositions.leftTrenchY;
            
            this.setControl(new SwerveRequest.FieldCentric()
            .withRotationalDeadband(MaxAngularRate * Constants.ControlConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(-xInput * MaxSpeed)
            .withVelocityY(-yController.calculate(pose.getY(), targetY))
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            );
        });
    }

    public Command holdAllignmentToClimb(CommandXboxController joystick){
        return this.run(() -> {
            double xInput = joystick.getLeftY();
            if (Math.abs(xInput) < Constants.ControlConstants.DEADBAND) xInput = 0;

            double targetY = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 3.730244 : 4.323588;
            this.setControl(new SwerveRequest.FieldCentric()
                .withRotationalDeadband(MaxAngularRate * Constants.ControlConstants.DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(xInput * -MaxSpeed)
                .withVelocityY(-yController.calculate(pose.getY(), targetY))
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            );
        });
    }

    

    public Command pointTowardsHub(CommandXboxController joystick) {
        return this.run(() -> {
            System.out.println("Pointing towards hub");
            boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
            double hubX = isBlue ? Constants.SwervePositions.blueHubX : Constants.SwervePositions.redHubX;
            double hubY = isBlue ? Constants.SwervePositions.blueHubY : Constants.SwervePositions.redHubY;
            
            double dx = hubX - pose.getX();
            double dy = hubY - pose.getY();
            Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx)).plus(new Rotation2d(Math.toRadians(isBlue ? Constants.SwervePositions.blueAngleOffset  : Constants.SwervePositions.redAngleOffset)));
            
            SmartDashboard.putNumber("dx", dx);
            SmartDashboard.putNumber("dy", dy);

            SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
            SmartDashboard.putNumber("Current Angle", pose.getRotation().getDegrees());

            int mult = isBlue ? -1 : 1;
            
            this.setControl(new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * Constants.ControlConstants.DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), Constants.ControlConstants.DEADBAND) * -MaxSpeed * mult)
                .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), Constants.ControlConstants.DEADBAND) * -MaxSpeed * mult)
                .withRotationalRate(yawController.calculate(pose.getRotation().getRadians(), targetAngle.getRadians()))
            );
        });
    }

    
    
    public Command autoPointTowardsHub() {
        return this.run(() -> {
            System.out.println("Pointing towards hub");
            boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
            double hubX = isBlue ? Constants.SwervePositions.blueHubX : Constants.SwervePositions.redHubX;
            double hubY = isBlue ? Constants.SwervePositions.blueHubY : Constants.SwervePositions.redHubY;
            
            double dx = hubX - pose.getX();
            double dy = hubY - pose.getY();
            Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));
            
            SmartDashboard.putNumber("dx", dx);
            SmartDashboard.putNumber("dy", dy);
            
            this.setControl(new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.ControlConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(
            yawController.calculate(pose.getRotation().getRadians(), targetAngle.getRadians())
            ));
        }).until(() -> {
            boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
            double hubX = isBlue ? Constants.SwervePositions.blueHubX : Constants.SwervePositions.redHubX;
            double hubY = isBlue ? Constants.SwervePositions.blueHubY : Constants.SwervePositions.redHubY;
            
            Rotation2d targetAngle = new Rotation2d(Math.atan2(hubY - pose.getY(), hubX - pose.getX()));
            double errorRadians = targetAngle.minus(pose.getRotation()).getRadians();
            double errorDegrees = Math.abs(Math.toDegrees(errorRadians));
            System.out.println("Yaw error: " + errorDegrees);
            return errorDegrees < 10;
        });
    }

    public Command pointTowardsAngle(CommandXboxController joystick, Rotation2d targetAngle) {
        return this.run(() -> {            
            yawController.setSetpoint(targetAngle.getRadians());
            this.setControl(new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.ControlConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), Constants.ControlConstants.DEADBAND) * MaxSpeed)
            .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), Constants.ControlConstants.DEADBAND) * MaxSpeed)
            .withRotationalRate(
            yawController.calculate(pose.getRotation().getRadians(), targetAngle.getRadians())));
        });
    }
    
    public Command toggleSlowMode() {
        return runOnce(() -> slowModeEnabled = !slowModeEnabled);
    }
    
    
    public Command resetHeading() {
        return runOnce(() -> resetRotation(getState().Pose.getRotation().rotateBy(addedRotation)));
    }
    
    
    
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }
    
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public ChassisSpeeds getSwerveSpeeds() {
        return kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    
    @Override
    public void periodic() {
        addedRotation = DriverStation.getAlliance()
    .map(a -> a == Alliance.Blue ? kBlueAlliancePerspectiveRotation : kRedAlliancePerspectiveRotation)
    .orElse(kBlueAlliancePerspectiveRotation);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {setOperatorPerspectiveForward(
                allianceColor == Alliance.Blue ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = false;
            });
        }
        
        //updateOdometry();
        if(odometry != null){
            pose = odometry.getPose();
        }
        
        SmartDashboard.putNumber("Odometry/Target X", AutoBuilder.getCurrentPose().getX());
        SmartDashboard.putNumber("Odometry/Target Y", AutoBuilder.getCurrentPose().getY());
        SmartDashboard.putBoolean("Odometry/Pointing Hub", yawController.atSetpoint());

        SmartDashboard.putNumber("Odometry/Hub Dx", Odometry.getHubDxDy()[0]);
        SmartDashboard.putNumber("Odometry/Hub Dy", Odometry.getHubDxDy()[1]);

        SmartDashboard.putBoolean("Swerve/Slow Mode", slowModeEnabled);

    }
    
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}