package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Shooter {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public DcMotorEx shooterMotorOne ;
    //public DcMotor  shooterMotorTwo;
    public CRServo triggerServo;
    private double targetVelocity = 100;
    private boolean triggerLockout = false;

    public Shooter(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
        initHardware();
    }

    protected void initHardware() {
//        shooterMotorOne = new MotorEx(hardwareMap, "MotorOne"); //intakeMotor = hardwareMap.get(MotorEx.class, "IntakeOne");
//        shooterMotorOne.setInverted(true); //.setDirection(DcMotor.Direction.FORWARD);
//        shooterMotorOne.stopMotor();
//        shooterMotorOne.resetEncoder(); //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterMotorOne.setRunMode(MotorEx.RunMode.RawPower); //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //shooterMotorOne.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT); //setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        //shooterMotorOne.setVeloCoefficients(1.789, 0.178, 0.0);
//        //shooterMotorOne.setFeedforwardCoefficients(0.92, 0.47);

        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "MotorOne");
        shooterMotorOne.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorOne.setPower(0.0);
        shooterMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "MotorOne");
        //shooterMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);
        //shooterMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //shooterMotorTwo.setPower(0.0);

        triggerServo = hardwareMap.get(CRServo.class, "ShooterServo");
        triggerServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void run() {
        shooterMotorOne.setPower(1.0);
    }

    public boolean isBusy() {
        if (shooterMotorOne.getVelocity() >= 0.0) {
            return true;
        }
        return false;
    }

    public void stop() {
        shooterMotorOne.setPower(0.0);
    }

    public void reverse() {
        shooterMotorOne.setPower(-1.0);
    }

    public void reset() {
        initHardware();
    }

    public void triggerRun() {
        triggerServo.setPower(0.5);
    }

    public void triggerStop() {
        triggerServo.setPower(0.0);
    }

    public void triggerBackOff(){
        triggerServo.setPower(-0.1);
    }


}