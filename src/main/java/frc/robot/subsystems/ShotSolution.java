package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShotSolution {
    private double timeOfFlight;
    private double distance;
    private double velocity;
    private Rotation2d angle;

    public ShotSolution(double TOF, double d, double v){
        timeOfFlight = TOF;
        distance = d;
        velocity = v;
    }

    public double getTimeOfFlight(){return timeOfFlight;}
    public double getDistance(){return distance;}
    public double getVelocity(){return velocity;}
    public Rotation2d getAngle(){return angle;}
    
    
    public void setRotation(Rotation2d a){angle=a;}
}
