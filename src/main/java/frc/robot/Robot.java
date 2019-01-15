/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.component.DriveBase;
import frc.robot.path.Path;
import frc.robot.path.PathCache;

public class Robot extends TimedRobot {
    private final DriveBase drive = new DriveBase();

    private final PathCache pathCache = new PathCache();
    private final SendableChooser<Path> pathSelector =
            new SendableChooser<>();

    public static Robot newRobot() {
        return new Robot();
    }

    @Override
    public void robotInit() {
        System.out.println("Robot initialization");

        this.pathCache.init();
        this.pathCache.populate(this.pathSelector);
    }

    @Override
    public void autonomousInit() {
        System.out.println("Auton initialization");

        this.drive.queuePath(this.pathSelector.getSelected());
    }

    @Override
    public void autonomousPeriodic() {
        this.drive.tickPath();
    }

    @Override
    public void teleopInit() {
        System.out.println("Teleop initialization");
    }

    @Override
    public void teleopPeriodic() {
    }
}