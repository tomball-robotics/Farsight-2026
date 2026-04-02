package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LimelightHelpers;

public class Odometry extends SubsystemBase{
    
    public SwerveDrivePoseEstimator poseEstimator;
    private Swerve swerve;
    
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    
    private final Rotation2d addedRotation = DriverStation.getAlliance()
    .map(a -> a == Alliance.Blue ? kBlueAlliancePerspectiveRotation : kRedAlliancePerspectiveRotation)
    .orElse(kBlueAlliancePerspectiveRotation);
    
    private static final String[] LIMELIGHT_NAMES = {"limelight-front", "limelight-left"};
    
    private final StructPublisher<Pose3d> pose3DPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Pose3D", Pose3d.struct).publish();
    private final StructPublisher<Pose2d> pose2DPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Pose2D", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();
    
    static Pose2d pose;
    
    public Odometry(Swerve drivetrain){
        
        poseEstimator = new SwerveDrivePoseEstimator(
        drivetrain.getKinematics(),
        drivetrain.getState().Pose.getRotation().rotateBy(Rotation2d.k180deg),
        drivetrain.getState().ModulePositions,
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
        swerve = drivetrain;
    }
    
    
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(swerve.getPigeon2().getRotation2d().plus(addedRotation), swerve.getState().ModulePositions, pose);
    }
    
    public void updateOdometry() {
        poseEstimator.update(swerve.getPigeon2().getRotation2d(), swerve.getState().ModulePositions);
        
        updateVisionMeasurements();
    }
    
    private void updateVisionMeasurements() {
        boolean anyLimelightBooted = false;
        poseEstimator.update(swerve.getPigeon2().getRotation2d().plus(addedRotation), swerve.getState().ModulePositions);
        SmartDashboard.putNumber("Yaw", swerve.getPigeon2().getRotation2d().getDegrees());
        SmartDashboard.putNumber("estimated Yaw", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
        for (String limelightName : LIMELIGHT_NAMES) {
            try {
                
                LimelightHelpers.SetRobotOrientation(limelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

                LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                
                // no tags detected
                if (poseEstimate.tagCount == 0) {
                    SmartDashboard.putBoolean("Limelight/" + limelightName + "/Has Targets", false);
                    continue;
                }
                
                boolean rejectUpdate = false;
                
                // filtering for single-tag measurements
                if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
                    if (poseEstimate.rawFiducials[0].ambiguity > 0.7 || poseEstimate.rawFiducials[0].distToCamera > 3) {
                        rejectUpdate = true;
                    }
                }
                
                if (!rejectUpdate) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.9, 0.9, 9999999));
                    poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                }
                
                anyLimelightBooted = true;
                SmartDashboard.putBoolean("Limelight/" + limelightName + "/Connected", true);
                SmartDashboard.putBoolean("Limelight/" + limelightName + "/Has Targets", true);
                SmartDashboard.putNumber("Limelight/" + limelightName + "/Tag Count", poseEstimate.tagCount);
                SmartDashboard.putBoolean("Limelight/" + limelightName + "/Update Rejected", rejectUpdate);
            } catch (Exception e) {
                SmartDashboard.putBoolean("Limelight/" + limelightName + "/Connected", false);
                SmartDashboard.putBoolean("Limelight/" + limelightName + "/Has Targets", false);
            }
        }
        
        SmartDashboard.putBoolean("Limelight Booted", anyLimelightBooted);
    }
    
    public Pose2d getPose(){
        return pose;
    }
    
    public double distanceToHub() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        double hubX = isBlue ? Constants.SwervePositions.blueHubX : Constants.SwervePositions.redHubX;
        double hubY = isBlue ? Constants.SwervePositions.blueHubY : Constants.SwervePositions.redHubY;
        return Math.hypot(hubX - pose.getX(), hubY - pose.getY());
    }
    
    public static double[] getHubDxDy(){
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        double hubX = isBlue ? Constants.SwervePositions.blueHubX : Constants.SwervePositions.redHubX;
        double hubY = isBlue ? Constants.SwervePositions.blueHubY : Constants.SwervePositions.redHubY;
        return new double[]{hubX - pose.getX(), hubY - pose.getY()};
    }
    
    public static boolean pointingTowardsHub() {
        double[] dxdy = getHubDxDy();
        double angleToHub = Math.atan2(dxdy[1], dxdy[0]);
        double robotAngle = pose.getRotation().getRadians();
        double angleDifference = Math.abs(angleToHub - robotAngle);
        return angleDifference < Math.toRadians(5); // within 5 degrees
    }
    
    @Override
    public void periodic(){
        updateOdometry();
        
        pose = poseEstimator.getEstimatedPosition();
        
        pose3DPublisher.set(new Pose3d(pose));
        pose2DPublisher.set(pose);
        chassisSpeedsPublisher.set(swerve.getState().Speeds);
        
        SmartDashboard.putNumber("Odometry/Distance to Hub", distanceToHub());
        SmartDashboard.putBoolean("Odometry/Pointing Towards Hub", pointingTowardsHub());


    }
}
