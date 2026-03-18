package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShotCalculator {
    InterpolatingTree solutionMap;

    public ShotCalculator(){
        solutionMap = new InterpolatingTree();
    }

    public ShotSolution solveShot(double dx, double dy, double xVelocity, double yVelocity, double angularVelocity){

        ShotSolution out = null;
        double distance = Math.hypot(dx, dy);

        double vx = xVelocity - dy * angularVelocity;
        double vy = yVelocity + dx * angularVelocity;
        
        if(Math.hypot(vx, vy) < 0.1){
            out = solutionMap.get(distance);
            out.setRotation(new Rotation2d(dx, dy));
        }
        else{
            double time = solutionMap.get(distance).getTimeOfFlight();

            for(int i = 0; i < 3; i++){
                double prx = dx - vx * time;
                double pry = dy - vy * time;
                distance = Math.hypot(prx, pry);

                double lookupTime = solutionMap.get(distance).getTimeOfFlight();
                double error = lookupTime - time;

                double derivativeDistance = -(prx * vx + pry * vy) / distance;
                double derivativeTime = (solutionMap.get(distance + 0.1).getTimeOfFlight() - solutionMap.get(distance - 0.1).getTimeOfFlight()) / 0.2;
                double derivative = derivativeDistance * derivativeTime - 1;

                if (Math.abs(derivative) > 1e-3) {
                    time -= error / derivative;
                } else {
                    time = lookupTime;
                }

                time = Math.max(0.05, Math.min(time, 2.0));
            }

            double prx = dx - vx * time;
            double pry = dy - vy * time;
            distance = Math.hypot(prx, pry);
            
           out =  solutionMap.get(distance);
           out.setRotation(new Rotation2d(prx, pry));
        }
        return out;

        
        
    }
}


class InterpolatingTree{
    ArrayList<ShotSolution> tunedSolutions;

    public InterpolatingTree(){
        tunedSolutions = new ArrayList<ShotSolution>();
    }

    public void addSolution(double distance, double velocity, double time){
        tunedSolutions.add(new ShotSolution(time, distance, velocity));
        Collections.sort(tunedSolutions, (s1, s2) -> {return Double.compare(s1.getDistance(), s2.getDistance());});
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