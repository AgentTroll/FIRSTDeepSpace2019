/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.sandstorm.SandstormGroup;
import frc.robot.component.Controller;
import frc.robot.component.NavX;
import frc.robot.component.NavX.NavXListener;
import frc.robot.component.RobotDrive;
import frc.robot.path.PathCache;
import frc.robot.path.PlannedPath;
import frc.robot.util.RobotLogger;
import lombok.Getter;

import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot {
    private static final Map<Integer, Integer> angleMapping = new HashMap<>(4, 1);
    static {
        angleMapping.put(45, 30);
        angleMapping.put(135, 150);
        angleMapping.put(225, 210);
        angleMapping.put(315, 330);
    }

    @Getter
    private final RobotLogger logger = new RobotLogger();

    private final PathCache pathCache = new PathCache(this);
    @Getter
    private final RobotDrive drive = new RobotDrive(this);
    @Getter
    private final NavX navX = new NavX(this);
    private final Controller controller = new Controller();

    private final SendableChooser<PlannedPath> pathSelector = new SendableChooser<>();
    private final SandstormGroup autoCommands = new SandstormGroup(this);

    private boolean wasPreviouslyRunning;

    private boolean hadDoublePOV; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

    private final NavXListener rotRateSrc = new NavXListener() {
        @Override
        protected void accept(double normalizedAngle) {
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    };

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

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setVideoMode(new VideoMode(PixelFormat.kMJPEG, 500, 500, 10));

        this.wasPreviouslyRunning = true;
        this.pathCache.init();
        this.pathCache.populate(this.pathSelector);

        this.drive.init();
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

        // J - Seems to be debug code
        /* this.navX.reset();
        this.navX.beginAction(0, null); */

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

        // J - Seems to be debug code
        /* this.drive.drive(0, 0, this.rotateToAngleRate, 0);
        this.navX.printTelemetry(); */
    }

    @Override
    public void teleopInit() {
        this.logger.log("Teleop initialization");

        this.autoCommands.cancel();
        this.drive.clearPaths();

        this.drive.setNeutralMode(NeutralMode.Coast);

        this.navX.reset();
        this.navX.beginAction(0, this.rotRateSrc);
    }

    @Override
    public void teleopPeriodic() {
        this.checkReset();//Resets Gyro When you press A
        this.checkCentricToggle();
        this.checkCollision();

        if (!this.navX.isConnected()) {
            this.isFieldCentric = false;
        }

        if (this.controller.getLeftTrigger() > 0.1 || this.controller.getRightTrigger() > 0.1) {
            this.strafe();
            this.checkSnapToAngle();
        } else if (this.isFieldCentric) {
            if (Math.sqrt(Math.pow(this.controller.getXRight(), 2) + Math.pow(this.controller.getYRight(), 2)) > 0.8) {
                this.correctTargetAngle();
            }
            this.drive.drive(-this.controller.getXLeft(), this.controller.getYLeft(),
                    this.rotRateSrc.getOutput(), this.navX.getYaw() - 2 * this.navX.getSetpoint());
            this.checkSnapToAngle();
        } else {
            this.drive.drive(-this.controller.getXLeft(), this.controller.getYLeft(), this.controller.getXRight(), 0);
        }

    }

    public void checkReset() {
        if (this.controller.isAPressed()) {
            this.navX.reset();
            this.navX.obtrudeSetpoint(0);
        }
    }

    public void checkCentricToggle() {
        if (this.controller.isStartPressed()) {
            this.isFieldCentric = !this.isFieldCentric;
        }
    }

    public void checkSnapToAngle() {
        int controllerPOV = this.controller.getPov();
        if (controllerPOV == -1) { // no Controller POV pressed
            this.hadDoublePOV = false;
            return;
        }

        controllerPOV = angleMapping.getOrDefault(controllerPOV, controllerPOV); // re-map angle to be correct
        controllerPOV = (controllerPOV > 180) ? controllerPOV - 360 : controllerPOV; // adjust range to (-180, 180]

        if (controllerPOV % 90 != 0) { // Controller POV is at mixed angle
            this.hadDoublePOV = true;
            this.navX.obtrudeSetpoint(controllerPOV);
        } else if (!this.hadDoublePOV) { // Controller POV only has one pressed AND no double POV pressed yet
            this.navX.obtrudeSetpoint(controllerPOV);
        }
    }

    public void correctTargetAngle() {
        double x = this.controller.getXRight();
        double y = this.controller.getYRight();

        double angle = Math.atan(y / x) * 180 / Math.PI;
        if (x < 0) {
            angle += 180;
        }

        double diff = 270 - angle;

        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;

        double correction = diff / 400; // TODO: magic number

        double newPoint = this.navX.getSetpoint() - correction;
        if (newPoint > 180) newPoint -= 360;
        if (newPoint <= -180) newPoint += 360;

        this.navX.obtrudeSetpoint(newPoint);
    }

    public void strafe() {
        double left = this.controller.getLeftTrigger();
        double right = this.controller.getRightTrigger();

        double power = left - right;
        if (Math.abs(power) > 0.05 && this.isFieldCentric) {
            this.drive.drive(power, 0,
                    this.rotRateSrc.getOutput(), this.navX.getYaw() - this.navX.getSetpoint());
            this.drive.drive(power, 0, this.rotRateSrc.getOutput(), 0);
        } else if (Math.abs(power) > 0.05) {
            this.drive.drive(power, 0, 0, 0);
        }
    }

    public void checkCollision() {
        boolean collisionDetected = this.navX.hasCollided();
        SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
        if (collisionDetected) {
            this.controller.beginRumble();
        } else {
            this.controller.stopRumble();
        }
    }
}