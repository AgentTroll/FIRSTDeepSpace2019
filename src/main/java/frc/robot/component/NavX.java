package frc.robot.component;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.Getter;

public class NavX implements PIDOutput {
    private static final long TIMEOUT_MS = 5_000;

    private final Robot robot;
    @Getter
    private final AHRS dev;

    private final PIDController controller;

    @Getter
    private double output;

    public NavX(Robot robot) {
        this.robot = robot;
        this.dev = new AHRS(SerialPort.Port.kMXP);
        this.dev.setPIDSourceType(PIDSourceType.kDisplacement);

        this.controller = new PIDController(0.0085, 0.00, 0.005, 0.00, this.dev, this);
        this.controller.setInputRange(-180.0, 180.0);
        this.controller.setOutputRange(-1, 1);
        this.controller.setAbsoluteTolerance(2.0);
        this.controller.setContinuous(true);
        this.controller.enable();
        this.controller.setSetpoint(0);
    }

    public void reset() {
        this.robot.getLogger().log("Resetting the NavX...");
        this.awaitCalibration();
        this.dev.reset();
        this.awaitCalibration();
        this.robot.getLogger().log("Reset complete.");
    }

    private void awaitCalibration() {
        long beginTime = System.currentTimeMillis();

        boolean connected;
        boolean calibrating;
        boolean hasShifted;
        do {
            connected = this.dev.isConnected();
            calibrating = this.dev.isCalibrating();
            hasShifted = this.dev.isMoving() || this.dev.isRotating();

            long elapsed = System.currentTimeMillis() - beginTime;
            if (elapsed > TIMEOUT_MS) {
                this.robot.getLogger().error("Calibration has taken longer than " + TIMEOUT_MS + "ms, proceeding anyways...");
                break;
            }
        } while (!connected || calibrating || hasShifted);
    }

    public boolean isConnected() {
        return this.dev.isConnected();
    }

    public double getYaw() {
        return this.dev.getYaw();
    }

    public double getAngle() {
        return this.dev.getAngle();
    }

    public boolean isMoving() {
        return this.dev.isMoving();
    }

    public void begin() {
        this.controller.enable();
    }

    public void beginAction(double angle) {
        this.controller.setSetpoint(angle);
        this.controller.enable();
    }

    public void obtrudeSetpoint(double setpoint) {
        this.controller.setSetpoint(setpoint);
    }

    public void printTelemetry() {
        SmartDashboard.putNumber("Get Angle", this.dev.getAngle());
        SmartDashboard.putBoolean("NavX Connected", this.dev.isConnected());

        SmartDashboard.putNumber("Target", this.controller.getSetpoint());
        SmartDashboard.putNumber("Set Point", this.controller.getSetpoint());
        SmartDashboard.putBoolean("onTarget", this.controller.onTarget());

        SmartDashboard.putNumber("Rotate to Angle Rate", this.output);
    }

    @Override
    public void pidWrite(double output) {
        if (this.controller.onTarget()) {
            this.output = 0;
        } else {
            this.output = output;
        }
    }
}
