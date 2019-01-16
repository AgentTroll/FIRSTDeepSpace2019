package frc.robot.auto.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.component.RobotDrive;
import frc.robot.path.PlannedPath;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class MovementCommand extends Command {
    private final Robot robot;
    private final PlannedPath path;

    @Override
    protected void initialize() {
        RobotDrive drive = this.robot.getDrive();
        drive.queuePath(this.path);
    }

    @Override
    protected void execute() {
        RobotDrive drive = this.robot.getDrive();
        drive.tickPath();
    }

    @Override
    protected boolean isFinished() {
        return this.path.isComplete();
    }
}
