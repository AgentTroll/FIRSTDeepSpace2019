package frc.robot.auto.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import lombok.RequiredArgsConstructor;

// TODO: Implement
@RequiredArgsConstructor
public class ReleaseHatchCommand extends Command {
    private final Robot robot;

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
