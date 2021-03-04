package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

import androidx.annotation.NonNull;

public class Lift {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    public DigitalChannel liftStopStore;
    public DcMotor liftMotor;
    public LiftClaw liftClaw;
    public boolean liftStopStoreSensor;

    public Lift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        liftClaw = new LiftClaw(hardwareMap, opMode);
        initHardware();
    }

    protected void initHardware() {
        liftStopStore = hardwareMap.get(DigitalChannel.class, "liftStopStore");
        liftStopStore.setMode(DigitalChannel.Mode.INPUT);
        liftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean liftStopStoreState() {
        return !liftStopStore.getState();
    }

    //SHould only be used in an iterative opmode, will cause damage robot otherwise!
    public void calibrate() {
        if (liftStopStoreState()) {
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(-0.4);
        }
    }

    public void setDropPrepPosition() {
        liftMotor.setTargetPosition(1500);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (liftMotor.getCurrentPosition() > 1000) {
            //If we go full speed we will overshoot and hit the sensor. Slow down when we get close!
            liftMotor.setPower(0.3);
        } else {
            liftMotor.setPower(1.0);
        }
    }

    public void setDropSoftPosition() {
        liftMotor.setTargetPosition(1700);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.2);
    }

    public void setDropPosition() {
        liftMotor.setTargetPosition(1700);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.8);
    }

    public void setStorePosition() {
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (liftMotor.getCurrentPosition() < 400) {
            //If we go full speed we will overshoot and hit the sensor. Slow down when we get close!
            liftMotor.setPower(0.3);
        } else {
            liftMotor.setPower(1.0);
        }
        if (liftStopStoreState() || !isBusy()) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public boolean isBusy() {
        return liftMotor.isBusy();
    }

    public void stop() {
        liftMotor.setPower(0);
    }

    public void reset() {
        initHardware();
    }

    public class LiftClaw {
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        public Servo liftClawServo;

        public LiftClaw(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        protected void initHardware() {
            liftClawServo = hardwareMap.get(Servo.class, "LiftClaw");
            liftClawServo.resetDeviceConfigurationForOpMode();
        }

        public void open() {
            liftClawServo.setPosition(0.0);
        }

        public void close() {
            liftClawServo.setPosition(1.0);
        }

        public void reset() {
            open();
        }
    }
}





