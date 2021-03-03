package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Shooter {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase opMode;
    public DcMotor shooterMotorOne;
    //public DcMotor  shooterMotorTwo;
    public CRServo triggerServo;
    private double targetVelocity = 100;
    private boolean triggerLockout = false;

    public Shooter(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        initHardware();
    }

    protected void initHardware() {
        shooterMotorOne = hardwareMap.get(DcMotor.class, "MotorOne");
        shooterMotorOne.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorOne.setPower(0);

        //shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "MotorTwo");
        //shooterMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);
        //shooterMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //shooterMotorTwo.setPower(0.0);

        triggerServo = hardwareMap.get(CRServo.class, "ShooterServo");
    }

    public void run() {
        shooterMotorOne.setPower(0.4);
    }

    public void triggerrun() {
//        if (triggerLockout && (shooterMotorOne.getVelocity()<targetVelocity*.95)) {
//            //Shooter trigger can only run when shooter is up to speed!
//            triggerServo.setPower(0.0);
//        } else {
            triggerServo.setPower(-0.5);
//        }
    }

    public void triggerstop() {
        //For this to work, shooter servo must be a continuous rotation servo.
        triggerServo.setPower(0.0);
    }

    public void stop() {
        shooterMotorOne.setPower(0.0);
        triggerstop();
    }

    public void reset() {
        initHardware();
    }

}