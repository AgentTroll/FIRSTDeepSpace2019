package frc.robot.component;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.Integrator;
import lombok.RequiredArgsConstructor;

import static frc.robot.RobotMap.*;

@RequiredArgsConstructor
public class Accelerometer {
    private final Robot robot;

    private final AnalogInput posAccelX = new AnalogInput(POS_ACCEL_X);
    private final AnalogInput negAccelX = new AnalogInput(NEG_ACCEL_X);
    private final AnalogInput posAccelY = new AnalogInput(POS_ACCEL_Y);
    private final AnalogInput negAccelY = new AnalogInput(NEG_ACCEL_Y);

    private final Integrator integrator = new Integrator();

    private double xAccel;
    private double yAccel;
    private double xError;
    private double yError;

    public void init() {
        AnalogInput.setGlobalSampleRate(200);

        this.posAccelX.setOversampleBits(12);
        this.negAccelX.setOversampleBits(12);
        this.posAccelY.setOversampleBits(12);
        this.negAccelY.setOversampleBits(12);

        long count = 0;
        while (Timer.getFPGATimestamp() < 10) {
            this.xError += (this.posAccelX.getVoltage() - this.negAccelX.getVoltage());
            this.yError += (this.posAccelY.getVoltage() - this.negAccelY.getVoltage());
            count++;
        }
        this.xError /= count;
        this.yError /= count;
    }

    public void tick() {
        this.xAccel = (this.posAccelX.getVoltage() - this.negAccelX.getVoltage() - this.xError - .0161) / 1.989;
        this.yAccel = (this.posAccelY.getVoltage() - this.negAccelY.getVoltage() - this.yError + .002) / 1.984;

        boolean moving = this.robot.getNavX().isMoving();
        this.integrator.updateDisplacement(this.xAccel, this.yAccel, 50, moving);
    }

    public void printTelemetry() {
        SmartDashboard.putNumber("X Displacement", this.integrator.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", this.integrator.getDisplacementY());
        SmartDashboard.putNumber("xAccel", this.xAccel);
        SmartDashboard.putNumber("yAccel", this.yAccel);
        SmartDashboard.putNumber("X Error", (this.xError - .0161) / 1.989);
        SmartDashboard.putNumber("Y Error", (this.yError + .002) / 1.984);
        SmartDashboard.putNumber("Positive Average", this.posAccelX.getAverageValue());
        SmartDashboard.putNumber("Negative Average", -this.negAccelX.getAverageValue());
        SmartDashboard.putNumber("Acceleration Average", (this.posAccelX.getAverageVoltage() / 1.989 - this.negAccelX.getAverageVoltage() / 1.989));
        SmartDashboard.putNumber("Acceleration", (this.posAccelX.getVoltage() - this.negAccelX.getVoltage()) / 1.989);
    }
}
