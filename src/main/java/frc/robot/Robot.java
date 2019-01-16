/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.sandstorm.SandstormGroup;
import frc.robot.component.RobotDrive;
import frc.robot.path.PathCache;
import frc.robot.path.PlannedPath;
import frc.robot.util.RobotLogger;
import lombok.Getter;

public class Robot extends TimedRobot {
    @Getter
    private final RobotLogger logger = new RobotLogger();

    @Getter
    private final RobotDrive drive = new RobotDrive(this);

    private final PathCache pathCache = new PathCache(this);
    private final SendableChooser<PlannedPath> pathSelector =
            new SendableChooser<>();
    private CommandGroup autoCommands;

    public static Robot newRobot() {
        return new Robot();
    }

    @Override
    public void robotInit() {
        this.logger.log("Robot initialization");

        this.pathCache.init();
        this.pathCache.populate(this.pathSelector);
    }

    @Override
    public void autonomousInit() {
        this.logger.log("Auton initialization");

        PlannedPath path = this.pathSelector.getSelected();
        if (path != null) {
            this.autoCommands = new SandstormGroup(this, path);
            this.autoCommands.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        this.logger.log("Teleop initialization");

        this.autoCommands.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }
}