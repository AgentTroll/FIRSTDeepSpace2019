package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.RequiredArgsConstructor;

import static frc.robot.RobotMap.*;

@RequiredArgsConstructor
public class Arms {
    private static final double INTAKE_CURRENT_LIMIT = 12.5;
    private static final int INTAKE_CURRENT_LIMIT_HARD = 15;

    private final Robot robot;

    private final WPI_TalonSRX leftArm = new WPI_TalonSRX(LEFT_ARM);
    private final WPI_TalonSRX rightArm = new WPI_TalonSRX(RIGHT_ARM);

    private final DoubleSolenoid armsDeploy = new DoubleSolenoid(SOLENOID, ARM_DEPLOY_FWD, ARM_DEPLOY_INV);
    private final WPI_TalonSRX intake = new WPI_TalonSRX(INTAKE);

    private boolean intakeActive;
    private boolean armsUp = true;

    public void init() {
        this.intake.configPeakCurrentLimit(0);
        this.intake.configContinuousCurrentLimit(INTAKE_CURRENT_LIMIT_HARD);
    }

    public void checkSpin() {
        if (this.intakeActive) {
            this.leftArm.set(-.28);
            this.rightArm.set(-.28);
        } else {
            this.leftArm.set(0);
            this.rightArm.set(0);
        }
    }

    public void checkIntake() {
        SecondaryController controller = this.robot.getSecondaryController();

        if (controller.getBButton()) {
            this.intakeActive = !this.intakeActive;
        }

        if (this.intake.getOutputCurrent() > INTAKE_CURRENT_LIMIT) {
            this.intakeActive = false;
            this.armsUp = true;
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
        SmartDashboard.putNumber("intake current", this.intake.getOutputCurrent());
    }
}
