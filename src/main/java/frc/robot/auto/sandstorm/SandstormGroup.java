package frc.robot.auto.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.path.PlannedPath;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class SandstormGroup extends CommandGroup {
    private final Robot robot;

    public void init(PlannedPath path) {
        this.addSequential(new MovementCommand(this.robot, path));
        this.addSequential(new ReleaseHatchCommand(this.robot));
    }
}
