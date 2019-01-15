package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.path.Path;

import static frc.robot.RobotMap.*;

public class DriveBase {
    private final MecanumDrive drive;

    private Path path;

    public DriveBase() {
        WPI_TalonSRX left1 = new WPI_TalonSRX(LEFT_MOTOR_1);
        WPI_TalonSRX left2 = new WPI_TalonSRX(LEFT_MOTOR_2);
        WPI_TalonSRX right1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
        WPI_TalonSRX right2 = new WPI_TalonSRX(RIGHT_MOTOR_2);

        this.drive = new MecanumDrive(left1, left2, right1, right2);
    }

    public void queuePath(Path path) {
        this.path = path;
        this.path.reset();
    }

    public void tickPath() {
        // TODO: Follow path
    }

    public void stop() {
        this.drive.stopMotor();
    }
}
