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
    private final PIDController strafeController = new PIDController(2.0, 0, 0, 0, this.autoStrafer, this.autoStrafer);

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
    private double zeroCenterX;
    private double zeroCenterY;
    private double oneCenterX;
    private double oneCenterY;
    private double contours;

    static Robot newRobot() {
        return new Robot();
    }

    // -----------------------------------------------------

    @Override
    public void robotInit() {
        this.drive.init();
        this.elevator.init();

        this.compressor.clearAllPCMStickyFaults();
        this.compressor.setClosedLoopControl(true);

        // this.strafeController.setInputRange(-1000, 1000);
        this.strafeController.setOutputRange(-1, 1);
        this.strafeController.setAbsoluteTolerance(5);
        this.strafeController.setContinuous(false);
        this.strafeController.enable();
        this.strafeController.setSetpoint(0);
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
        this.arms.runIntake();
        this.checkToggleCentric();

        if (this.primaryController.isAPressed()) {
            this.autoStrafer.run();
            return;
        }

        if (!this.navX.isConnected()) {
            this.isFieldCentric = false;
        }

        if (this.isFieldCentric) {
            this.driveCentric();
            this.snapToAngle();
        } else {
            this.driveStandard();
        }
    }

    // Robot status ----------------------------------------

    private void checkReset() {
        if (this.primaryController.isXPressed()) {
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

    private void driveCentric() {
        this.drive.drive(-this.primaryController.getXLeft(), this.primaryController.getYLeft(), this.navX.getOutput(), -this.navX.getAngle());
    }

    private void driveStandard() {
        this.drive.drive(this.primaryController.getXLeft(), -this.primaryController.getYLeft(), this.primaryController.getXRight(), 0);
    }

    private void strafeCentric(double val) { //TODO: test
        this.drive.drive(val, this.primaryController.getYLeft(), this.navX.getOutput(), -this.navX.getAngle());
    }

    // Telemetry/comms -------------------------------------

    private void readNetTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        NetworkTableEntry zeroX = table.getEntry("zeroX");
        NetworkTableEntry zeroY = table.getEntry("zeroY");
        NetworkTableEntry oneX = table.getEntry("oneX");
        NetworkTableEntry oneY = table.getEntry("oneY");
        NetworkTableEntry contours = table.getEntry("contoursCount");
        inst.startServer();
        inst.setServerTeam(4131);

        this.zeroCenterX = zeroX.getDouble(0);
        this.zeroCenterY = zeroY.getDouble(0);
        this.oneCenterX = oneX.getDouble(0);
        this.oneCenterY = oneY.getDouble(0);
        this.contours = contours.getDouble(0);
    }

    private void updateTelemetry() {
        this.navX.printTelemetry();
        this.arms.printTelemetry();

        SmartDashboard.putBoolean("isCentric", this.isFieldCentric);

        SmartDashboard.putNumber("zeroX", this.zeroCenterX);
        SmartDashboard.putNumber("zeroY", this.zeroCenterY);
        SmartDashboard.putNumber("oneX", this.oneCenterX);
        SmartDashboard.putNumber("oneY", this.oneCenterY);
        SmartDashboard.putNumber("contours", this.contours);
    }

    private class AutoStrafer implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut;

        void run() {
            Robot.this.strafeCentric(this.pidOut);
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
            if (this.pidSourceType == PIDSourceType.kDisplacement) {
                // assumes 0 is center of frame
                return (160 - (Robot.this.zeroCenterX + Robot.this.oneCenterX) / 2) / (Robot.this.oneCenterX - Robot.this.zeroCenterX);
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }
}
