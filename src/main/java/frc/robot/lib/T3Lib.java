package frc.robot.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class T3Lib {
    
    private static final double DEFAULT_SUPPLY_CURRENT_LIMIT = 40.0;
    private static final double DEFAULT_STATOR_CURRENT_LIMIT = 60.0;
    private static final int DEFAULT_CONFIG_ATTEMPTS = 5;
    
    /**
    * Attempts to apply the given config to the given TalonFX up to maxAttempts times,
    * and reports an error if it fails after maxAttempts attempts
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
            "[T3Lib] Failed to apply config to TalonFX ID " + motor.getDeviceID() +
            " after " + maxAttempts + " attempts", false);
        }
    }
    
    /**
    * Attempts to apply the given config to the given Pigeon2 up to maxAttempts times,
    * and reports an error if it fails after maxAttempts attempts
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
            "[T3Lib] Failed to apply config to Pigeon2 ID " + pigeon.getDeviceID() +
            " after " + maxAttempts + " attempts", false);
        }
    }
    
    /**
    * Creates a TalonFX on the RIO bus with default current limits (40A supply, 60A stator)
    */
    public static TalonFX createTalonFX(int id, NeutralModeValue neutralMode, boolean inverted) {
        return createTalonFX(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT);
    }
    
    /**
    * Creates a TalonFX on the RIO bus with explicit current limits
    */
    public static TalonFX createTalonFX(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
        return createTalonFX(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
    }
    /**
     * Creates a TalonFX on the RIO bus with only supply current limit
     */

    public static TalonFX createTalonFX(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit) {
        return createTalonFX(id, null, neutralMode, inverted, supplyCurrentLimit, DEFAULT_STATOR_CURRENT_LIMIT);
    }
    
    /**
    * Creates a TalonFX on the specified CANivore bus with explicit current limits
    */
    public static TalonFX createTalonFX(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
        return motor;
    }
    
    /**
    * Creates a TalonFX configured for velocity PID on the RIO bus with default current limits (40A supply, 60A stator)
    */
    public static TalonFX createTalonFXVelocity(int id, NeutralModeValue neutralMode, boolean inverted, double kP, double kI, double kD, double kV) {
        return createTalonFXVelocity(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD, kV);
    }
    
    /**
    * Creates a TalonFX configured for velocity PID on the RIO bus with explicit current limits
    */
    public static TalonFX createTalonFXVelocity(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD, double kV) {
        return createTalonFXVelocity(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit, kP, kI, kD, kV);
    }
    
    /**
    * Creates a TalonFX configured for velocity PID on the specified CANivore bus with explicit current limits
    */
    public static TalonFX createTalonFXVelocity(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD, double kV) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
        return motor;
    }
    
    /**
    * Creates a TalonFX configured for position PID on the RIO bus with default current limits (40A supply, 60A stator)
    */
    public static TalonFX createTalonFXPosition(int id, NeutralModeValue neutralMode, boolean inverted, double kP, double kI, double kD) {
        return createTalonFXPosition(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD);
    }
    
    /**
    * Creates a TalonFX configured for position PID on the RIO bus with explicit current limits
    */
    public static TalonFX createTalonFXPosition(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD) {
        return createTalonFXPosition(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit, kP, kI, kD);
    }

    /**
    * Creates a TalonFX configured for position PID on the RIO bus with explicit supply current limit
    */
    public static TalonFX createTalonFXPosition(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double kP, double kI, double kD) {
        return createTalonFXPosition(id, null, neutralMode, inverted, supplyCurrentLimit, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD);
    }
    
    /**
    * Creates a TalonFX configured for position PID on the specified CANivore bus with explicit current limits
    */
    public static TalonFX createTalonFXPosition(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
        return motor;
    }
        
    private static TalonFXConfiguration buildBaseConfig(NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        return config;
    }
}