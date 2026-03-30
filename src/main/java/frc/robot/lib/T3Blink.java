package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class T3Blink extends SubsystemBase {

    private static final int BLINKIN_PWM_PORT = 0;
    private static final Spark blinkin = new Spark(BLINKIN_PWM_PORT);

    private static Pattern current = Pattern.BLACK;

    public enum Pattern {
        RAINBOW_RAINBOW(-0.99),
        RAINBOW_PARTY(-0.97),
        RAINBOW_OCEAN(-0.95),
        RAINBOW_LAVA(-0.93),
        RAINBOW_FOREST(-0.91),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),
        SINELON_RAINBOW(-0.79),
        SINELON_PARTY(-0.77),
        SINELON_OCEAN(-0.75),
        SINELON_LAVA(-0.73),
        SINELON_FOREST(-0.71),
        BPM_RAINBOW(-0.69),
        BPM_PARTY(-0.67),
        BPM_OCEAN(-0.65),
        BPM_LAVA(-0.63),
        BPM_FOREST(-0.61),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW(-0.55),
        TWINKLES_PARTY(-0.53),
        TWINKLES_OCEAN(-0.51),
        TWINKLES_LAVA(-0.49),
        TWINKLES_FOREST(-0.47),
        COLOR_WAVES_RAINBOW(-0.45),
        COLOR_WAVES_PARTY(-0.43),
        COLOR_WAVES_OCEAN(-0.41),
        COLOR_WAVES_LAVA(-0.39),
        COLOR_WAVES_FOREST(-0.37),
        LARSON_SCANNER_RED(-0.35),
        LARSON_SCANNER_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),

        COLOR1_BLEND_TO_BLACK(-0.03),
        COLOR1_LARSON_SCANNER(-0.01),
        COLOR1_LIGHT_CHASE(0.01),
        COLOR1_HEARTBEAT_SLOW(0.03),
        COLOR1_HEARTBEAT_MEDIUM(0.05),
        COLOR1_HEARTBEAT_FAST(0.07),
        COLOR1_BREATH_SLOW(0.09),
        COLOR1_BREATH_FAST(0.11),
        COLOR1_SHOT(0.13),
        COLOR1_STROBE(0.15),

        COLOR2_BLEND_TO_BLACK(0.17),
        COLOR2_LARSON_SCANNER(0.19),
        COLOR2_LIGHT_CHASE(0.21),
        COLOR2_HEARTBEAT_SLOW(0.23),
        COLOR2_HEARTBEAT_MEDIUM(0.25),
        COLOR2_HEARTBEAT_FAST(0.27),
        COLOR2_BREATH_SLOW(0.29),
        COLOR2_BREATH_FAST(0.31),
        COLOR2_SHOT(0.33),
        COLOR2_STROBE(0.35),

        SPARKLE_COLOR1_ON_COLOR2(0.37),
        SPARKLE_COLOR2_ON_COLOR1(0.39),
        COLOR_GRADIENT(0.41),
        BPM_COLOR1_AND_COLOR2(0.43),
        BLEND_COLOR1_TO_COLOR2(0.45),
        BLEND_COLOR1_AND_COLOR2(0.47),
        COLOR1_AND_COLOR2_NO_BLEND(0.49),
        TWINKLES_COLOR1_AND_COLOR2(0.51),
        COLOR_WAVES_COLOR1_AND_COLOR2(0.53),
        SINELON_COLOR1_AND_COLOR2(0.55),

        HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        RED_ORANGE(0.63),
        ORANGE(0.65),
        GOLD(0.67),
        YELLOW(0.69),
        LAWN_GREEN(0.71),
        LIME(0.73),
        DARK_GREEN(0.75),
        GREEN(0.77),
        BLUE_GREEN(0.79),
        AQUA(0.81),
        SKY_BLUE(0.83),
        DARK_BLUE(0.85),
        BLUE(0.87),
        BLUE_VIOLET(0.89),
        VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        BLACK(0.99);

        public final double value;

        Pattern(double value) {
            this.value = value;
        }
    }

    public static void setFor(double seconds, Pattern strobePattern) {
        Pattern previous = current;
        set(strobePattern);
        new Thread(() -> {
            Timer.delay(seconds);
            set(previous);
        }).start();
    }

    public static void setFor(double seconds, Pattern strobePattern, Pattern endPattern) {
        set(strobePattern);
        new Thread(() -> {
            Timer.delay(seconds);
            set(endPattern);
        }).start();
    }

    public static void setDefault() {
        set(Pattern.RAINBOW_RAINBOW);
    }

    private static void set(double value) {
        blinkin.set(value);
    }

    public static void set(Pattern pattern) {
        set(pattern.value);
    }

    public static void setRaw(double value) {
        set(value);
    }

    public static void off() {
        set(Pattern.BLACK);
    }
}