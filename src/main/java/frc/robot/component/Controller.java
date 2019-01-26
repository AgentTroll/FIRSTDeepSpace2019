package frc.robot.component;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.wpilibj.GenericHID.Hand.kLeft;
import static edu.wpi.first.wpilibj.GenericHID.Hand.kRight;
import static frc.robot.RobotMap.CONTROLLER;

@RequiredArgsConstructor
public class Controller {
    private final XboxController controller = new XboxController(CONTROLLER);

    public boolean isAPressed() {
        // TODO: Verify the right button
        return this.controller.getRawButton(1);
    }

    public boolean isStartPressed() {
        return this.controller.getStartButtonPressed();
    }

    public double getLeftTrigger() {
        return this.controller.getTriggerAxis(kLeft);
    }

    public double getRightTrigger() {
        return this.controller.getTriggerAxis(kRight);
    }

    public double getXRight() {
        return this.controller.getX(kRight);
    }

    public double getXLeft() {
        return this.controller.getX(kLeft);
    }

    public double getYRight() {
        return this.controller.getY(kRight);
    }

    public double getYLeft() {
        return this.controller.getY(kLeft);
    }

    public int getPov() {
        return this.controller.getPOV();
    }

    public void beginRumble() {
        this.controller.setRumble(RumbleType.kLeftRumble, 1);
        this.controller.setRumble(RumbleType.kRightRumble, 1);
    }

    public void stopRumble() {
        this.controller.setRumble(RumbleType.kLeftRumble, 0);
        this.controller.setRumble(RumbleType.kRightRumble, 0);
    }
}
