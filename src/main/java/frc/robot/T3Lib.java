package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class T3Lib {
    public static void applyConfig(TalonFX motor, TalonFXConfiguration config){
        boolean done = false;
        while(!done){
            done = motor.getConfigurator().apply(config).value == 0;
        }
    }
}
