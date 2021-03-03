package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Intake {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase opMode;
    public  DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        initHardware();
    }

    protected void initHardware() {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeOne");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
    }

    public void run() {
        intakeMotor.setPower(.8);
    }

    public boolean isBusy() {
        return intakeMotor.isBusy();
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reset() {
        initHardware();
    }

    public void reverse() { intakeMotor.setPower(-1);
    }

}