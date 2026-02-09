// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
  }

  public static final class ShooterConstants{
    public static final int SHOOTER_VELOCITY_MOTOR_ID = -1; //TODO tune
    public static final int SHOOTER_ANGLE_MOTOR_ID = -1; //TODO tune
    public static final double VOLTS_TO_MPS = 0.0; //TODO calculate
    public static final double TICKS_TO_DEGREES = 0.0; //TODO calculate

  }

  public static final class TreadmillConstants{
    public static final int TREADMILL_ID = -1; //TODO tune
    public static final double TREADMILL_SPEED = 6.7; //TODO tune
  }

  public static final class IndexerConstants{
    public static final int INDEXER_ID = -1; //TODO tune
    public static final double INDEXER_SPEED = 6.7; //TODO tune
  }

  public static final class ClimberConstants{
    public static final int CLIMBER_ID = -1; //TODO tune
    public static final double TOP_POSITION = -1; //TODO tune
    public static final double MAX_LENGTH = -1; //TODO tune
    public static final double CLIMBER_SPEED = 6.7; //TODO tune

  }

  public static final class IntakePivotConstants{
    public static final int INTAKE_PIVOT_ID = -1; //TODO tune
    public static final double DOWN_POSITION = -1; //TODO tune
  }

  public static final class IntakeRollerConstants {
    public static final int INTAKE_ROLLERS_ID = -1; //TODO tune
    public static final double INTAKE_ROLLERS_SPEED = 6.7; //TODO tune
  }

}
