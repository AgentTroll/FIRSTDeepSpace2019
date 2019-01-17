package frc.robot.component;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Robot;
import lombok.Getter;

import javax.annotation.Nullable;

public class NavX implements PIDOutput {
    private final Robot robot;
    @Getter
    private final AHRS dev;

    private final PIDController controller;

    @Nullable
    private Listener listener;

    public NavX(Robot robot) {
        this.robot = robot;
        this.dev = new AHRS(Port.kMXP);

        // TODO: Figure out the PID values
        this.controller = new PIDController(0.05, 0, 0, 0, this.dev, this);
        this.controller.setInputRange(-180, 180);
        this.controller.setAbsoluteTolerance(5);
        this.controller.setContinuous(false);
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
        boolean connected;
        boolean calibrating;
        boolean hasShifted;
        do {
            connected = this.dev.isConnected();
            calibrating = this.dev.isCalibrating();
            hasShifted = this.dev.isMoving() || this.dev.isRotating();
        } while (!connected || calibrating || hasShifted);
    }

    public double getYaw() {
        return this.dev.getYaw();
    }

    public void beginAction(double angle, Listener listener) {
        Listener current = this.listener;
        if (current != null) {
            this.robot.getLogger().error("NavX listener " + current + " is being replaced by " + listener);
            this.robot.getLogger().dumpStack();

            this.controller.disable();
        }

        this.controller.setSetpoint(angle);
        this.listener = listener;
        this.controller.enable();
    }

    public void cancelAction(Listener listener) {
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

    @Override
    public void pidWrite(double output) {
        if (this.listener != null) {
            if (this.listener.writeAngle(output)) {
                this.cancelAction();
            }
        }
    }

    public abstract static class Listener {
        @Getter
        private double lastAngleNormalized;

        final boolean writeAngle(double newAngle) {
            this.lastAngleNormalized = newAngle;
            this.accept(newAngle);

            return this.isFinished();
        }

        protected abstract void accept(double normalizedAngle);

        protected abstract boolean isFinished();
    }
}
