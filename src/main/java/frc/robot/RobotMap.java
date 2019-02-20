package frc.robot;

public final class RobotMap {
    // Controllers
    public static final int PRIMARY_CONTROLLER = 0;
    public static final int SECONDARY_CONTROLLER = 1;

    // Accelerometer ports
    public static final int POS_ACCEL_X = 0;
    public static final int NEG_ACCEL_X = 1;
    public static final int POS_ACCEL_Y = 2;
    public static final int NEG_ACCEL_Y = 3;

    // Motor ports to initialize the drive base
    public static final int LEFT_MOTOR_1 = 1;
    public static final int LEFT_MOTOR_2 = 2;
    public static final int RIGHT_MOTOR_1 = 3;
    public static final int RIGHT_MOTOR_2 = 4;

    public static final int SOLENOID = 61;

    // Arms/intake
    public static final int LEFT_ARM = 5;
    public static final int RIGHT_ARM = 6;
    public static final int ARM_DEPLOY_FWD = 4;
    public static final int ARM_DEPLOY_INV = 5;
    public static final int INTAKE = 8;

    // Elevator
    public static final int ELEVATOR = 7;

    // Hatch mechanism
    public static final int HATCH_DEPLOY_FWD = 2;
    public static final int HATCH_DEPLOY_INV = 3;
    public static final int HATCH_FWD = 0;
    public static final int HATCH_INV = 1;

    // Suppress instantiation
    private RobotMap() {
    }
}
