/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.sandstorm.SandstormGroup;
import frc.robot.component.NavX;
import frc.robot.component.RobotDrive;
import frc.robot.path.PathCache;
import frc.robot.path.PlannedPath;
import frc.robot.util.RobotLogger;
import lombok.Getter;

public class Robot extends TimedRobot {
    @Getter
    private final RobotLogger logger = new RobotLogger();

    private final PathCache pathCache = new PathCache(this);
    @Getter
    private final RobotDrive drive = new RobotDrive(this);
    @Getter
    private final NavX navX = new NavX(this);

    private final SendableChooser<PlannedPath> pathSelector = new SendableChooser<>();
    private final SandstormGroup autoCommands = new SandstormGroup(this);

    private boolean wasPreviouslyRunning;

    public static Robot newRobot() {
        return new Robot();
    }

    @Override
    public void robotInit() {
        this.logger.log("Robot initialization");

        this.reset();

        this.logger.log("wasPreviouslyRunning = " + this.wasPreviouslyRunning);
        if (this.wasPreviouslyRunning) {
            return;
        }

        this.wasPreviouslyRunning = true;
        this.pathCache.init();
        this.pathCache.populate(this.pathSelector);
    }

    private void reset() {
        this.logger.log("Resetting the robot...");
        this.navX.cancelAction();
        this.drive.clearPaths();

        this.navX.reset();
    }

    @Override
    public void autonomousInit() {
        this.logger.log("Auton initialization");

        PlannedPath path = this.pathSelector.getSelected();
        if (path != null) {
            this.autoCommands.init(path);
            this.autoCommands.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Run the autoCommands group
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        this.logger.log("Teleop initialization");

        this.autoCommands.cancel();
        this.drive.clearPaths();
    }

    @Override
    public void teleopPeriodic() {
    }
}