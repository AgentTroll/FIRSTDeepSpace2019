package frc.robot.component;

import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.wpilibj.GenericHID.Hand.kLeft;
import static edu.wpi.first.wpilibj.GenericHID.Hand.kRight;
import static frc.robot.RobotMap.SECONDARY_CONTROLLER;

public class SecondaryController {
    private final XboxController controller = new XboxController(SECONDARY_CONTROLLER);

    public boolean getAButtonPressed() {
        return this.controller.getRawButtonPressed(1);
    }

    public boolean getAButtonReleased() {
        return this.controller.getRawButtonReleased(1);
    }

    public boolean getBButton() {
        return this.controller.getBButtonReleased();
    }

    public boolean getXButton() {
        return this.controller.getRawButton(3);
    }

    public boolean getYButton() {
        return this.controller.getRawButton(4);
    }

    public boolean getLeftBumper() {
        return this.controller.getRawButtonReleased(5);
    }

    public double getLeftTrigger() {
        return this.controller.getTriggerAxis(kLeft);
    }

    public double getRightTrigger() {
        return this.controller.getTriggerAxis(kRight);
    }

    public boolean getStartButton() {
        return this.controller.getStartButtonReleased();
    }
}
