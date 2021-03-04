package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;


public class ArcRoboticsDrive {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    private MecanumDrive mecanumDrive;
    protected Util util;
    public Motor leftFrontWheel;
    public Motor rightFrontWheel;
    public Motor leftRearWheel;
    public Motor rightRearWheel;
    public boolean turtleMode = false;

    public ArcRoboticsDrive(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        initHardware();
    }

    protected void initHardware() {
        leftFrontWheel = new Motor(hardwareMap, "LeftFrontWheel");
        leftFrontWheel.setInverted(true);
        leftFrontWheel.stopMotor();
        leftFrontWheel.resetEncoder();
        leftFrontWheel.setRunMode(Motor.RunMode.RawPower);
        leftFrontWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightFrontWheel = new Motor(hardwareMap, "RightFrontWheel");
        rightFrontWheel.setInverted(false);
        rightFrontWheel.stopMotor();
        rightFrontWheel.resetEncoder();
        rightFrontWheel.setRunMode(Motor.RunMode.RawPower);
        rightFrontWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftRearWheel = new Motor(hardwareMap, "LeftRearWheel");
        leftRearWheel.setInverted(true);
        leftRearWheel.stopMotor();
        leftRearWheel.resetEncoder();
        leftRearWheel.setRunMode(Motor.RunMode.RawPower);
        leftRearWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightRearWheel = new Motor(hardwareMap, "RightRearWheel");
        rightRearWheel.setInverted(false);
        rightRearWheel.stopMotor();
        rightRearWheel.resetEncoder();
        rightRearWheel.setRunMode(Motor.RunMode.RawPower);
        rightRearWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //mecanumDrive = new MecanumDrive(leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel);
        turtleMode = false;
    }

    /*
     * basicDrive takes inputs directly from gamepad and translates them to chassis movement.
     * This is sometimes called an arcade drive.
     */
    public void driveRobotCentric(double moveX, double moveY, double moveRotation) {
        //setTurtleMode();
        //mecanumDrive.driveRobotCentric(moveX, moveY, moveRotation);
    }

    public void driveFieldCentric(double moveX, double moveY, double moveRotation, double heading) {
        //setTurtleMode();
        //mecanumDrive.driveFieldCentric(moveX, moveY, moveRotation, heading);
    }

    public void setTurtleMode() {
//        if (turtleMode) {
//            mecanumDrive.setMaxSpeed(0.5);
//        } else {
//            mecanumDrive.setMaxSpeed(1.0);
//        }
    }

    public void stop() {
        //mecanumDrive.stop();
    }

    public void reset() {

    }
}
