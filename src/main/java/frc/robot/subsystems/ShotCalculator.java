package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ShotCalculator {
    public static InterpolatingList solutionMap = new InterpolatingList();

    

    public static double warmStart = -1;

    static
    {
        //Add Tuned Solutions here
        solutionMap.addSolution(1.0, 5, 1);
    }

    public static ShotSolution solveShot(double dx, double dy, double xVelocity, double yVelocity, double angularVelocity){

        ShotSolution out = null;
        double distance = Math.hypot(dx, dy);

        double vx = xVelocity - dy * angularVelocity;
        double vy = yVelocity + dx * angularVelocity;
        
        if(Math.hypot(vx, vy) < Constants.ShotSolutionConstants.VELOCITY_MIN){
            out = solutionMap.get(distance);
            out.setRotation(new Rotation2d(dx, dy));
        }
        else{
            double time;
            if(warmStart < 0){
                time = solutionMap.get(distance).getTimeOfFlight();
            }
            else{
                time = warmStart;
            }

            for(int i = 0; i < Constants.ShotSolutionConstants.ITERATIONS; i++){
                double prx = dx - vx * time;
                double pry = dy - vy * time;
                distance = Math.hypot(prx, pry);

                double lookupTime = solutionMap.get(distance).getTimeOfFlight();
                double error = lookupTime - time;

                double derivativeDistance = -(prx * vx + pry * vy) / distance;
                double derivativeTime = (solutionMap.get(distance + Constants.ShotSolutionConstants.EPSILON).getTimeOfFlight() - solutionMap.get(distance - Constants.ShotSolutionConstants.EPSILON).getTimeOfFlight()) / (2*Constants.ShotSolutionConstants.EPSILON);
                double derivative = derivativeDistance * derivativeTime - 1;

                if (Math.abs(derivative) > Constants.ShotSolutionConstants.DERIVATIVE_MINIMUM) {
                    time -= error / derivative;
                } else {
                    time = lookupTime;
                }

                time = Math.max(Constants.ShotSolutionConstants.MINIMUM_TOF, Math.min(time, Constants.ShotSolutionConstants.MAXIMUM_TOF));
                warmStart = time;
            }

            double prx = dx - vx * time;
            double pry = dy - vy * time;
            distance = Math.hypot(prx, pry);
            
           out =  solutionMap.get(distance);
           out.setRotation(new Rotation2d(prx, pry));
        }
        return out;
    }

    public static ShotSolution stationary(double distance){
        return solutionMap.get(distance);
    }
}


class InterpolatingList{
    ArrayList<ShotSolution> tunedSolutions;

    public InterpolatingList(){
        tunedSolutions = new ArrayList<ShotSolution>();
    }

    public void addSolution(double distance, double velocity, double time){
        int index = 0;
        while(index < tunedSolutions.size() && tunedSolutions.get(index).getDistance() < distance){index++;}
        tunedSolutions.add(index, new ShotSolution(time, distance, velocity));
    }

    public ShotSolution get(double distance){
        if (distance <= tunedSolutions.get(0).getDistance()) return tunedSolutions.get(0);
        if (distance >= tunedSolutions.get(tunedSolutions.size() - 1).getDistance()) return tunedSolutions.get(tunedSolutions.size() - 1);
    
        for (int i = 0; i < tunedSolutions.size() - 1; i++) {
            ShotSolution lo = tunedSolutions.get(i);
            ShotSolution hi = tunedSolutions.get(i + 1);
            if (distance >= lo.getDistance() && distance <= hi.getDistance()) {
                double velocity = interpolate(lo.getDistance(), hi.getDistance(), lo.getVelocity(), hi.getVelocity(), distance);
                double time = interpolate(lo.getDistance(), hi.getDistance(), lo.getTimeOfFlight(), hi.getTimeOfFlight(), distance);
                return new ShotSolution(distance, velocity, time);
            }
        }
        return tunedSolutions.get(tunedSolutions.size() - 1);
    }

    private double interpolate(double x1, double x2, double y1, double y2, double x) {
        if (x1 == x2) {return y1;}
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}