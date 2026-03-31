package frc.robot.lib.T3Lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class T3Util {

    /**
     * Attempts to apply the given config to the given TalonFX up to maxAttempts times,
     * and reports an error if it fails after maxAttempts attempts.
     */
    public static void applyConfig(TalonFX motor, TalonFXConfiguration config, int maxAttempts) {
        boolean success = false;
        for (int i = 0; i < maxAttempts; i++) {
            success = motor.getConfigurator().apply(config).value == 0;
            if (success) break;
            Timer.delay(0.1);
        }
        if (!success) {
            DriverStation.reportError(
                "[T3Util] Failed to apply config to TalonFX ID " + motor.getDeviceID() +
                " after " + maxAttempts + " attempts", false);
        }
    }

    /**
     * Attempts to apply the given config to the given Pigeon2 up to maxAttempts times,
     * and reports an error if it fails after maxAttempts attempts.
     */
    public static void applyConfig(Pigeon2 pigeon, Pigeon2Configuration config, int maxAttempts) {
        boolean success = false;
        for (int i = 0; i < maxAttempts; i++) {
            success = pigeon.getConfigurator().apply(config).value == 0;
            if (success) break;
            Timer.delay(0.1);
        }
        if (!success) {
            DriverStation.reportError(
                "[T3Util] Failed to apply config to Pigeon2 ID " + pigeon.getDeviceID() +
                " after " + maxAttempts + " attempts", false);
        }
    }
}