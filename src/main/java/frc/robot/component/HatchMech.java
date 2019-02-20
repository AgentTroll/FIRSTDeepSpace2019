package frc.robot.component;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import static frc.robot.RobotMap.*;

@RequiredArgsConstructor
public class HatchMech {
    private final Robot robot;

    private final DoubleSolenoid hatchDeploy = new DoubleSolenoid(SOLENOID, HATCH_DEPLOY_FWD, HATCH_DEPLOY_INV);
    private final DoubleSolenoid hatchMechanism = new DoubleSolenoid(SOLENOID, HATCH_FWD, HATCH_INV);

    @Setter
    private boolean isHatchDown;

    public void checkDeploy() {
        SecondaryController controller = this.robot.getSecondaryController();

        if (this.isHatchDown) {
            this.hatchDeploy.set(DoubleSolenoid.Value.kForward);
        } else {
            this.hatchDeploy.set(DoubleSolenoid.Value.kReverse);
        }
        if (controller.getLeftBumper()) {
            this.isHatchDown = !this.isHatchDown;
        }
    }

    // TODO: wtf does this even do?
    public void check() {
        SecondaryController controller = this.robot.getSecondaryController();

        if (controller.getAButton()) {
            this.hatchMechanism.set(DoubleSolenoid.Value.kForward);
        } else {
            this.hatchMechanism.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
