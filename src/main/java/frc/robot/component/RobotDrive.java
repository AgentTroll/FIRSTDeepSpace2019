package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.path.PlannedPath;
import jaci.pathfinder.Trajectory.Segment;

import javax.annotation.Nonnull;
import java.util.Deque;
import java.util.LinkedList;

import static frc.robot.RobotMap.*;
import static jaci.pathfinder.Pathfinder.d2r;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class RobotDrive {
    private final Robot robot;
    private final MecanumDrive drive;

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
        this.robot.getLogger().log("Path " + path.getName() + " has been queued");
    }

    public void clearPaths() {
        this.pathQueue.clear();
        this.robot.getLogger().log("Cleared all paths");
    }

    public void tickPath() {
        PlannedPath path = this.pathQueue.peekFirst();
        if (path == null) {
            return;
        }

        if (path.isComplete()) {
            this.pathQueue.removeFirst();
            this.robot.getLogger().log("Completed path " + path.getName());
            this.tickPath();
            return;
        }

        Segment segment = path.getSegmentAndIncr();
        double magnitude = segment.velocity;
        double angle = segment.heading;

        NavX navX = this.robot.getNavX();
        navX.beginAction(angle, new NavX.Listener() {
            @Override
            protected void accept(double normalizedAngle) {
                double rad = d2r(angle);
                drive(magnitude * sin(rad),
                        magnitude * cos(rad),
                        normalizedAngle,
                        navX.getYaw());
            }

            @Override
            protected boolean isFinished() {
                // Only complete a single run
                return true;
            }
        });
    }

    public void drive(double y, double x, double z, double gyro) {
        this.drive.driveCartesian(y, x, z, gyro);
    }

    public void stop() {
        this.drive.stopMotor();
    }
}
