package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
  }
  public static final class SwervePositions{
    public static final double leftTrenchY = 0.650748; 
    public static final double rightTrenchY = 7.391908;
    

    public static final double redHubX = 11.915;
    public static final double redHubY = 4.021;

    public static final double blueHubX = 4.611624;
    public static final double blueHubY = 4.021;

  }
  public static final class ShooterConstants{
    public static final double SHOOTER_MAX_ANGLE = 0.0;
    public static final double SHOOTER_MIN_ANGLE = 11.84375;



    public static final int RIGHT_SHOOTER_MOTOR_ID = 5; 
    public static final int LEFT_SHOOTER_MOTOR_ID = 7; 


  }

  public static final class TreadmillConstants{
    public static final int TREADMILL_ID = 6; 
    public static final double TREADMILL_SPEED = 7; //TODO tune
  }

  public static final class IndexerConstants{
    public static final int INDEXER_ID = 3; 
    public static final double INDEXER_SPEED = 4; //TODO tune
  }

  public static final class ClimberConstants{
    public static final int CLIMBER_ID = -1; //TODO tune
    public static final double TOP_POSITION = -1; //TODO tune
    public static final double MAX_LENGTH = -1; //TODO tune
    public static final double CLIMBER_SPEED = 0; //TODO tune

  }

  public static final class IntakePivotConstants{
    public static final int INTAKE_PIVOT_LEADER_ID = 2; 
    public static final int INTAKE_PIVOT_FOLLOWER_ID = 4; 
    public static final double DOWN_POSITION = 3.31640625;//-0.7001953125; //TODO tune 
    public static final double UP_POSITION = 0; //0
  } 

  public static final class IntakeRollerConstants {
    public static final int INTAKE_ROLLERS_ID = 1; 
    public static final double INTAKE_ROLLERS_SPEED = 6.7; //TODO tune
  }
}
