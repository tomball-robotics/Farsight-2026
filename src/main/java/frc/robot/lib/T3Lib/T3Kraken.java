package frc.robot.lib.T3Lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class T3Kraken {

    private static final double DEFAULT_SUPPLY_CURRENT_LIMIT = 40.0;
    private static final double DEFAULT_STATOR_CURRENT_LIMIT = 60.0;
    private static final int DEFAULT_CONFIG_ATTEMPTS = 5;

    // ── Basic ────────────────────────────────────────────────────────────────

    /** Creates a TalonFX on the RIO bus with default current limits (40A supply, 60A stator). */
    public static TalonFX create(int id, NeutralModeValue neutralMode, boolean inverted) {
        return create(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT);
    }

    /** Creates a TalonFX on the RIO bus with an explicit supply current limit. */
    public static TalonFX create(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit) {
        return create(id, null, neutralMode, inverted, supplyCurrentLimit, DEFAULT_STATOR_CURRENT_LIMIT);
    }

    /** Creates a TalonFX on the RIO bus with explicit current limits. */
    public static TalonFX create(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
        return create(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
    }

    /** Creates a TalonFX on the specified CANivore bus with explicit current limits. */
    public static TalonFX create(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        T3Util.applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
        return motor;
    }

    /** Creates a TalonFX configured for velocity PID on the RIO bus with default current limits (40A supply, 60A stator). */
    public static TalonFX createVelocity(int id, NeutralModeValue neutralMode, boolean inverted, double kP, double kI, double kD, double kV) {
        return createVelocity(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD, kV);
    }

    /** Creates a TalonFX configured for velocity PID on the RIO bus with explicit current limits. */
    public static TalonFX createVelocity(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD, double kV) {
        return createVelocity(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit, kP, kI, kD, kV);
    }

    /** Creates a TalonFX configured for velocity PID on the specified CANivore bus with explicit current limits. */
    public static TalonFX createVelocity(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD, double kV) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        T3Util.applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
        return motor;
    }


    /** Creates a TalonFX configured for position PID on the RIO bus with default current limits (40A supply, 60A stator). */
    public static TalonFX createPosition(int id, NeutralModeValue neutralMode, boolean inverted, double kP, double kI, double kD) {
        return createPosition(id, null, neutralMode, inverted, DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD);
    }

    /** Creates a TalonFX configured for position PID on the RIO bus with an explicit supply current limit. */
    public static TalonFX createPosition(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double kP, double kI, double kD) {
        return createPosition(id, null, neutralMode, inverted, supplyCurrentLimit, DEFAULT_STATOR_CURRENT_LIMIT, kP, kI, kD);
    }

    /** Creates a TalonFX configured for position PID on the RIO bus with explicit current limits. */
    public static TalonFX createPosition(int id, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD) {
        return createPosition(id, null, neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit, kP, kI, kD);
    }

    /** Creates a TalonFX configured for position PID on the specified CANivore bus with explicit current limits. */
    public static TalonFX createPosition(int id, String canBus, NeutralModeValue neutralMode, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit, double kP, double kI, double kD) {
        TalonFX motor = canBus != null ? new TalonFX(id, canBus) : new TalonFX(id);
        TalonFXConfiguration config = buildBaseConfig(neutralMode, inverted, supplyCurrentLimit, statorCurrentLimit);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        T3Util.applyConfig(motor, config, DEFAULT_CONFIG_ATTEMPTS);
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