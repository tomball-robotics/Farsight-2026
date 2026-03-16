package frc.robot;

public final class Constants {
  
  public static class ControlConstants {
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
    public static final int RIGHT_SHOOTER_MOTOR_ID = 5; 
    public static final int LEFT_SHOOTER_MOTOR_ID = 7; 
    public static final int SUPPLY_CURRENT_LIMIT = 40;
  }
  
  public static final class ClimberConstants{
    public static final int LEFT_CLIMBER_ID = -1;
    public static final int RIGHT_CLIMBER_ID = -1;
    public static final double MAX_EXTENSION = -1;
    public static final double CLIMBER_SPEED = 0;
    public static final int CLIMBER_CURRENT_LIMIT = 40;
  }
  
  public static final class IntakeConstants{
    public static final int INTAKE_PIVOT_LEADER_ID = 2; 
    public static final int INTAKE_PIVOT_FOLLOWER_ID = 4; 
    public static final int INTAKE_ROLLERS_ID = 1; 

    public static final double DOWN_POSITION = 4.220215;
    public static final double UP_POSITION = 0;

    public static final double INTAKE_ROLLERS_SPEED = -6.7;
    
    public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 40;
    public static final int ROLLERS_SUPPLY_CURRENT_LIMIT = 40;
  } 
  
  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 6; 
    public static final double ROLLER_SPEED = 6.7;
    public static final int SUPPLY_CURRENT_LIMIT = 40;
  }
  
  public static final class FeederConstants {
    public static final int FEEDER_MOTOR_ID = 3;
    public static final int FEEDER_SPEED = 4;
    public static final int SUPPLY_CURRENT_LIMIT = 40;
  }
  
}
