package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DigitalChannel liftStopStore;
    public DigitalChannel liftStopDrop;
    public DcMotor liftMotor;
    public LiftClaw liftClaw;

    public Lift(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        liftClaw = new LiftClaw(hardwareMap, opMode);
        initHardware();
    }

    protected void initHardware() {
        liftStopStore = hardwareMap.get(DigitalChannel.class, "liftStopStore");
        liftStopStore.setMode(DigitalChannel.Mode.INPUT);
//        liftStopDrop = hardwareMap.get(DigitalChannel.class, "liftStopStore");
//        liftStopDrop.setMode(DigitalChannel.Mode.INPUT);

        liftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if (!liftStopStore.getState() || !liftMotor.isBusy()) {
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

        public LiftClaw(HardwareMap hardwareMap, OpMode opMode) {
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





