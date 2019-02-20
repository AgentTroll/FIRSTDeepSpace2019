/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.sandstorm.SandstormGroup;
import frc.robot.component.RobotDrive;
import frc.robot.component.*;
import frc.robot.path.PathCache;
import frc.robot.path.PlannedPath;
import frc.robot.util.RobotLogger;
import lombok.Getter;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.RobotMap.SOLENOID;

@Getter
public class Robot extends TimedRobot {
    //TODO: send vision center to network tables
    //too small - left too big - right
    private static final double VISION_CENTER_X = 164;
    private static final Map<Integer, Integer> ANGLE_MAP = new HashMap<>();

    static {
        ANGLE_MAP.put(45, 30);
        ANGLE_MAP.put(135, 150);
        ANGLE_MAP.put(225, 240);
        ANGLE_MAP.put(315, 330);
    }

    private final RobotLogger logger = new RobotLogger();

    private final PathCache pathCache = new PathCache(this);
    private final NavX navX = new NavX(this);
    private final Accelerometer accelerometer = new Accelerometer(this);
    private final Robot.AutoStrafer autoStrafer = new Robot.AutoStrafer();
    PIDController strafeController = new PIDController(.75, 0, 0.6, 0, this.autoStrafer, this.autoStrafer);

    Robot.AutoDriver autoDriver = new Robot.AutoDriver();
    PIDController driveController = new PIDController(0.003, 0, 0.01, 0, this.autoDriver, this.autoDriver);

    private final PrimaryController primaryController = new PrimaryController();
    private final SecondaryController secondaryController = new SecondaryController();

    private final Compressor compressor = new Compressor(SOLENOID);
    private final RobotDrive drive = new RobotDrive(this);
    private final Elevator elevator = new Elevator(this);
    private final Arms arms = new Arms(this);
    private final HatchMech hatchMech = new HatchMech(this);

    private final SendableChooser<PlannedPath> pathSelector = new SendableChooser<>();
    private final SandstormGroup autoCommands = new SandstormGroup(this);

    private boolean hadDoublePOV; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

    // the centers of the two retro-reflective targets, if both the x and y of a target are zero it isn't detected
    private double ballZeroCenterX;
    private double ballZeroCenterY;
    private double ballOneCenterX;
    private double ballOneCenterY;
    private double hatchZeroCenterX;
    private double hatchZeroCenterY;
    private double hatchOneCenterX;
    private double hatchOneCenterY;
    private double ballContours;
    private double hatchContours;

    static Robot newRobot() {
        return new Robot();
    }

    // -----------------------------------------------------

    @Override
    public void robotInit() {
        this.drive.init();
        this.arms.init();
        this.elevator.init();

        this.compressor.clearAllPCMStickyFaults();
        this.compressor.setClosedLoopControl(true);

        this.strafeController.setOutputRange(-0.5, 0.5);
        this.strafeController.setAbsoluteTolerance(5);
        this.strafeController.setContinuous(false);
        this.strafeController.enable();
        this.strafeController.setSetpoint(0);

        this.driveController.setOutputRange(-0.4, 0.4);
        this.driveController.setAbsoluteTolerance(5);
        this.driveController.setContinuous(false);
        this.driveController.enable();
        this.driveController.setSetpoint(240); // pixel distance between targets when fully in (- a bit)
    }

    @Override
    public void autonomousInit() {
        this.navX.begin();
        this.drive.setSafetyEnabled(true);
        this.hatchMech.setHatchDown(true);
    }

    @Override
    public void autonomousPeriodic() {
        while (this.isAutonomous()) {
            this.readNetTable();
            this.tickRobot();
            this.updateTelemetry();
            Timer.delay(.005);
        }
    }

    @Override
    public void teleopInit() {
        this.navX.begin();
        this.drive.setSafetyEnabled(true);
        this.hatchMech.setHatchDown(true);
    }

    @Override
    public void teleopPeriodic() {
        while (this.isOperatorControl()) { // avoid extra teleop baggage
            this.readNetTable();
            this.tickRobot();
            this.updateTelemetry();
            Timer.delay(.005); // a bit odd, but told to not remove
        }
    }

    private void tickRobot() {
        this.hatchMech.check();
        this.hatchMech.checkDeploy();
        this.checkReset();
        this.arms.checkSpin();
        this.elevator.checkAdjust();
        this.elevator.run();
        this.arms.checkIntake();
        this.checkToggleCentric();
        this.snapToAngle();

        if (this.primaryController.getAButton()) {
//            autoStrafer.run();
            this.autoStrafeDrive();
            return;
        }

        if (!this.navX.isConnected()) {
            this.isFieldCentric = false;
        }

        if (this.isFieldCentric) {
            this.driveCentric();
        } else {
            this.driveStandard();
        }
    }

    // Robot status ----------------------------------------

    private void checkReset() {
        if (this.primaryController.getXButton()) {
            this.navX.reset();
        }
    }

    private void checkToggleCentric() {
        if (this.primaryController.getLeftBumper()) {
            this.isFieldCentric = !this.isFieldCentric;
        }
    }

    // Drive control ---------------------------------------

    private void snapToAngle() {
        if (this.primaryController.getPov() == -1) { // no controller POV pressed
            this.hadDoublePOV = false;
            return;
        }

        int controllerPOV = this.primaryController.getPov();
        controllerPOV = ANGLE_MAP.getOrDefault(controllerPOV, controllerPOV);

        if (controllerPOV > 180) {
            controllerPOV -= 360;
        }

        if (controllerPOV % 90 != 0) { // controller POV is at mixed angle
            this.hadDoublePOV = true;
            this.navX.obtrudeSetpoint(controllerPOV);
        } else if (!this.hadDoublePOV) { // controller POV only has one pressed AND no double POV pressed yet
            this.navX.obtrudeSetpoint(controllerPOV);
        }
    }

    public void autoStrafeDrive() {
        if (this.isStrafeAligned())
            this.strafeCentric(this.autoStrafer.pidOut, this.autoDriver.pidOut);
        else
            this.strafeCentric(this.autoStrafer.pidOut, 0);
    }

    private boolean isStrafeAligned() {
        double offCenter = Math.abs(VISION_CENTER_X - (this.hatchZeroCenterX + this.hatchOneCenterX) / 2);
        return offCenter < 30;
    }

    public void driveCentric() {
        this.drive.drive(-this.primaryController.getXLeft(), this.primaryController.getYLeft(), this.navX.getOutput(), -this.navX.getAngle());
    }

    public void driveStandard() {
        this.drive.drive(this.primaryController.getXLeft(), -this.primaryController.getYLeft(), this.primaryController.getXRight(), 0);
    }

    public void strafeCentric(double x, double y) { //TODO: test
        this.drive.drive(-x, y, this.navX.getOutput(), 0); // y as xSpeed is correct
    }

    // Telemetry/comms -------------------------------------

    private void readNetTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        NetworkTableEntry ballZeroX = table.getEntry("ballZeroX");
        NetworkTableEntry ballZeroY = table.getEntry("ballZeroY");
        NetworkTableEntry ballOneX = table.getEntry("ballOneX");
        NetworkTableEntry ballOneY = table.getEntry("ballOneY");
        NetworkTableEntry ballContours = table.getEntry("ballContoursCount");
        NetworkTableEntry hatchZeroX = table.getEntry("hatchZeroX");
        NetworkTableEntry hatchZeroY = table.getEntry("hatchZeroY");
        NetworkTableEntry hatchOneX = table.getEntry("hatchOneX");
        NetworkTableEntry hatchOneY = table.getEntry("hatchOneY");
        NetworkTableEntry hatchContours = table.getEntry("hatchContoursCount");
        NetworkTableEntry debug = table.getEntry("debug");
        SmartDashboard.putNumber("debug", debug.getNumber(0).doubleValue());
        inst.startServer();
        inst.setServerTeam(4131);

        this.ballZeroCenterX = ballZeroX.getDouble(0);
        this.ballZeroCenterY = ballZeroY.getDouble(0);
        this.ballOneCenterX = ballOneX.getDouble(0);
        this.ballOneCenterY = ballOneY.getDouble(0);
        this.ballContours = ballContours.getDouble(0);
        this.hatchZeroCenterX = hatchZeroX.getDouble(0);
        this.hatchZeroCenterY = hatchZeroY.getDouble(0);
        this.hatchOneCenterX = hatchOneX.getDouble(0);
        this.hatchOneCenterY = hatchOneY.getDouble(0);
        this.hatchContours = hatchContours.getDouble(0);

        NetworkTableEntry dir = table.getEntry("strings");
        SmartDashboard.putString("dir", dir.getString("err"));
    }

    private void updateTelemetry() {
        this.navX.printTelemetry();
        this.arms.printTelemetry();

        SmartDashboard.putBoolean("isCentric", this.isFieldCentric);

        SmartDashboard.putNumber("ballZeroX", this.ballZeroCenterX);
        SmartDashboard.putNumber("ballZeroY", this.ballZeroCenterY);
        SmartDashboard.putNumber("ballOneX", this.ballOneCenterX);
        SmartDashboard.putNumber("ballOneY", this.ballOneCenterY);
        SmartDashboard.putNumber("ballContours", this.ballContours);
        SmartDashboard.putNumber("hatchZeroX", this.hatchZeroCenterX);
        SmartDashboard.putNumber("hatchZeroY", this.hatchZeroCenterY);
        SmartDashboard.putNumber("hatchOneX", this.hatchOneCenterX);
        SmartDashboard.putNumber("hatchOneY", this.hatchOneCenterY);
        SmartDashboard.putNumber("hatchContours", this.hatchContours);

        SmartDashboard.putNumber("pid for drive", this.autoDriver.pidOut);
        SmartDashboard.putNumber("center average", (this.hatchZeroCenterX + this.hatchOneCenterX) / 2);
        SmartDashboard.putNumber("center difference", this.hatchOneCenterX - this.hatchZeroCenterX);
    }

    private class AutoStrafer implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut;

        public void run() {
            Robot.this.strafeCentric(this.pidOut, Robot.this.primaryController.getYLeft());
        }

        @Override
        public void pidWrite(double output) {
            this.pidOut = output;
            SmartDashboard.putNumber("strafe val", this.pidOut);
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            this.pidSourceType = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return this.pidSourceType;
        }

        @Override
        public double pidGet() {
            double max = Math.max(Robot.this.hatchZeroCenterX, Robot.this.hatchOneCenterX);
            double min = Math.min(Robot.this.hatchZeroCenterX, Robot.this.hatchOneCenterX);
            if (this.pidSourceType == PIDSourceType.kDisplacement) {
                // assumes 0 is center of frame
                return (VISION_CENTER_X - (max + min) / 2) / (max - min);
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }

    private class AutoDriver implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut;

        @Override
        public void pidWrite(double output) {
            this.pidOut = -output;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            this.pidSourceType = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return this.pidSourceType;
        }

        @Override
        public double pidGet() {
            double max = Math.max(Robot.this.hatchZeroCenterX, Robot.this.hatchOneCenterX);
            double min = Math.min(Robot.this.hatchZeroCenterX, Robot.this.hatchOneCenterX);
            if (this.pidSourceType == PIDSourceType.kDisplacement) {
                return max - min;
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }
}
