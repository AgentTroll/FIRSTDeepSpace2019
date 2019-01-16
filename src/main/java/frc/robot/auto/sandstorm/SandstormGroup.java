package frc.robot.auto.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.path.PlannedPath;

public class SandstormGroup extends CommandGroup {
    public SandstormGroup(Robot robot, PlannedPath path) {
        this.addSequential(new MovementCommand(robot, path));
        this.addSequential(new ReleaseHatchCommand(robot));
    }
}
