package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.RequiredArgsConstructor;

import static frc.robot.RobotMap.*;

@RequiredArgsConstructor
public class Arms {
    private final Robot robot;

    private final WPI_TalonSRX leftArm = new WPI_TalonSRX(LEFT_ARM);
    private final WPI_TalonSRX rightArm = new WPI_TalonSRX(RIGHT_ARM);

    private final DoubleSolenoid armsDeploy = new DoubleSolenoid(SOLENOID, ARM_DEPLOY_FWD, ARM_DEPLOY_INV);
    private final WPI_TalonSRX intake = new WPI_TalonSRX(INTAKE);

    private boolean intakeActive;
    private boolean armsUp = true;

    public void checkSpin() {
        if (this.intakeActive) {
            this.leftArm.set(-.35);
            this.rightArm.set(.35);
        } else {
            this.leftArm.set(0);
            this.rightArm.set(0);
        }
    }

    public void runIntake() {
        SecondaryController controller = this.robot.getSecondaryController();

        if (controller.getBButton()) {
            this.intakeActive = !this.intakeActive;
        }

        if (this.intakeActive) {
            this.intake.set(1);
        } else if (controller.getLeftTrigger() > 0.05 || controller.getRightTrigger() > 0.05) {
            this.intake.set(-1);
        } else {
            this.intake.set(0);
        }

        if (controller.getStartButton()) {
            this.armsUp = !this.armsUp;
        }

        if (!this.armsUp) {
            this.armsDeploy.set(DoubleSolenoid.Value.kForward);
        } else {
            this.armsDeploy.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void printTelemetry() {
        SmartDashboard.putBoolean("Arms Up", this.armsUp);
    }
}
