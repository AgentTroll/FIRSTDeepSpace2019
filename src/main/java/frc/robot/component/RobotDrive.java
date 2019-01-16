package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.path.PlannedPath;
import jaci.pathfinder.Trajectory;

import javax.annotation.Nonnull;
import java.util.Deque;
import java.util.LinkedList;

import static frc.robot.RobotMap.*;

public class RobotDrive {
    private static final long PROGRESS_TIMEOUT = 5000L;

    private final Robot robot;
    private final MecanumDrive drive;

    private long lastProgressTime = System.currentTimeMillis();
    private final Deque<PlannedPath> pathQueue =
            new LinkedList<>();

    public RobotDrive(Robot robot) {
        this.robot = robot;

        WPI_TalonSRX left1 = new WPI_TalonSRX(LEFT_MOTOR_1);
        WPI_TalonSRX left2 = new WPI_TalonSRX(LEFT_MOTOR_2);
        WPI_TalonSRX right1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
        WPI_TalonSRX right2 = new WPI_TalonSRX(RIGHT_MOTOR_2);
        this.drive = new MecanumDrive(left1, left2, right1, right2);
    }

    public void queuePath(@Nonnull PlannedPath path) {
        path.reset();
        this.pathQueue.addLast(path);
    }

    public void popPath() {
        this.pathQueue.pollFirst();
    }

    public void clearPaths() {
        this.pathQueue.clear();
    }

    public void tickPath() {
        PlannedPath path = this.pathQueue.pollFirst();
        if (path == null) {
            return;
        }

        long currentTime = System.currentTimeMillis();
        if (path.isComplete()) {
            this.popPath();

            if (this.hasMadeProgress(currentTime)) {
                this.tickPath();
            }
            return;
        }

        this.updateProgress(currentTime);

        Trajectory.Segment segment = path.getSegmentAndIncr();

        // TODO: Complete this
        double magnitude = segment.velocity;
        double angle = segment.heading;
        // Stole off of MecanumDrive.java
        this.driveRaw(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                magnitude * Math.cos(angle * (Math.PI / 180.0)), 0, 0.0);
    }

    public void driveRaw(double y, double x, double z, double gyro) {
        this.drive.driveCartesian(y, x, z, gyro);
    }

    public void stop() {
        this.drive.stopMotor();
    }

    private boolean hasMadeProgress(long currentTime) {
        boolean isWithinTimeout = currentTime - this.lastProgressTime < PROGRESS_TIMEOUT;
        if (!isWithinTimeout) {
            this.robot.getLogger().error("Robot has timed out; dumping stack");
            new Exception().printStackTrace();
        }

        return isWithinTimeout;
    }

    private void updateProgress(long currentTime) {
        this.lastProgressTime = currentTime;
    }
}
