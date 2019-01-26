package frc.robot.component;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.Getter;

import javax.annotation.Nullable;
import javax.annotation.concurrent.GuardedBy;

public class NavX implements PIDOutput {
    private static final long TIMEOUT_MS = 5_000;
    private static final double COLLISION_G_THRESHOLD = 2f;

    private final Robot robot;
    @Getter
    private final AHRS dev;

    private final PIDController controller;

    private double lastXAccel;
    private double lastYAccel;

    @Nullable
    private volatile NavXListener listener;

    public NavX(Robot robot) {
        this.robot = robot;
        this.dev = new AHRS(Port.kMXP);
        this.dev.setPIDSourceType(PIDSourceType.kDisplacement);

        this.controller = new PIDController(0.01, 0, 0.05, 0, this.dev, this);
        this.controller.setInputRange(-180, 180);
        this.controller.setAbsoluteTolerance(2);
        this.controller.setContinuous(true);
        this.controller.setOutputRange(-1, 1);
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

    public void beginAction(double angle, @Nullable NavXListener listener) {
        NavXListener current = this.listener;
        if (current != null) {
            this.robot.getLogger().error("NavX listener " + current + " is being replaced by " + listener);
            this.robot.getLogger().dumpStack();

            this.controller.disable();
        }

        this.controller.setSetpoint(angle);
        this.listener = listener;
        this.controller.enable();
    }

    public void obtrudeSetpoint(double setpoint) {
        this.controller.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return this.controller.getSetpoint();
    }

    public void cancelAction(NavXListener listener) {
        if (this.listener == listener) {
            this.cancelAction();
        }
    }

    public void cancelAction() {
        if (this.controller.isEnabled()) {
            this.listener = null;
            this.controller.disable();
        }
    }

    public boolean hasCollided() {
        double curXAccel = this.dev.getWorldLinearAccelX();
        double currentJerkX = curXAccel - this.lastXAccel;
        this.lastXAccel = curXAccel;
        double curYAccel = this.dev.getWorldLinearAccelY();
        double currentJerkY = curYAccel - this.lastYAccel;
        this.lastYAccel = curYAccel;

        return Math.abs(currentJerkX) > COLLISION_G_THRESHOLD ||
                Math.abs(currentJerkY) > COLLISION_G_THRESHOLD;
    }

    public void printTelemetry() {
        SmartDashboard.putNumber("X Displacement", this.dev.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", this.dev.getDisplacementY());
    }

    @Override
    public void pidWrite(double output) {
        NavXListener listener = this.listener;
        if (listener != null) {
            if (listener.writeAngle(output)) {
                this.cancelAction();
            }
        }
    }

    public abstract static class NavXListener {
        @GuardedBy("this")
        private double output;

        final boolean writeAngle(double output) {
            synchronized (this) {
                this.output = output;
            }
            this.accept(output);

            return this.isFinished();
        }

        public double getOutput() {
            synchronized (this) {
                return this.output;
            }
        }

        protected abstract void accept(double normalizedAngle);

        protected abstract boolean isFinished();
    }
}
