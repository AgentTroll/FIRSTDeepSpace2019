package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static frc.robot.RobotMap.ELEVATOR;

@RequiredArgsConstructor
public class Elevator {
    /**
     * The elevation (inches) of the center of the lowest rocket cargo port
     */
    private static final double PORT_LOW = 27.5;

    /**
     * The distance (inches) between the centers of each rocket cargo port
     */
    private static final double PORT_INTERVAL = 28;

    /**
     * The elevation (inches) of the center of a cargo ball when it is in
     * the lowest position within the inner claw - i.e. just collected
     */
    private static final double LOWEST_CARGO_CENTER = 12; //TODO: evaluate

    /**
     * Encoder ticks per each inch the lifter travels vertically
     */
    private static final double TICKS_PER_VERTICAL_INCH = 1000; // TODO: evaluate

    /**
     * The elevation (ticks) of the point at which we want to eject cargo into the cargo ship.
     */
    public static final double CARGO_DEPOSIT_HEIGHT = 40 * TICKS_PER_VERTICAL_INCH; //TODO: evaluate

    private final Robot robot;

    @Getter
    private final WPI_TalonSRX talon = new WPI_TalonSRX(ELEVATOR);

    private double target;

    public void init() {
        this.talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.talon.setSensorPhase(true);

        this.talon.setInverted(false);

        this.talon.configNominalOutputForward(0);
        this.talon.configNominalOutputReverse(0);
        this.talon.configPeakOutputForward(0.5);
        this.talon.configPeakOutputReverse(-0.5);

        this.talon.configAllowableClosedloopError(0, 16);

        this.talon.enableCurrentLimit(true);
        this.talon.configContinuousCurrentLimit(20);
        this.talon.configPeakCurrentLimit(0);

        this.talon.config_kP(0, 0.5);
        this.talon.config_kI(0, 0);
        this.talon.config_kD(0, 0);
        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 30);
        this.talon.setSelectedSensorPosition(0);
    }

    public void run() {
        this.talon.set(ControlMode.Position, this.target);
    }

    public void checkAdjust() {
        this.printTelemetry();

        SecondaryController secondary = this.robot.getSecondaryController();
        if (secondary.getYButton()) {
            this.target = 11300;
        } else if (secondary.getXButton()) {
            this.target = 0;
        }
    }

    private void printTelemetry() {
        SmartDashboard.putNumber("elevator Encoder", this.talon.getSelectedSensorPosition());
    }
}
