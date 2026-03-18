// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.ShotSolution;
import frc.robot.subsystems.Swerve;

public class SetupShot extends Command {
  private Swerve swerve;
  private Shooter shooter;
  private Odometry odometry;
  CommandXboxController driver;


  public SetupShot(Swerve swerve, Shooter shooter, Odometry odometry, CommandXboxController driver) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.odometry = odometry;
    this.driver = driver;
    addRequirements(swerve, shooter, odometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] distances = odometry.getHubDxDy();
    ChassisSpeeds speeds = swerve.getSwerveSpeeds();
    ShotSolution solution = ShotCalculator.solveShot(distances[0], distances[1], speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

    shooter.setVelocity(solution.getVelocity());
    swerve.pointTowardsAngle(driver, solution.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
