package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RevBlinkinLedDriver blinkinLedDriver;

    public Lights(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        initHardware();
    }

    protected void initHardware() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setPattern(String patternName) {
        if (patternName != null && !patternName.trim().isEmpty()) {
            //telemetry.addData("Light Pattern:", patternName);
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(patternName));
        } else {
            //telemetry.addData("Light Pattern NONE:", patternName);
        }
    }

    public void stop() {
        setPattern("BLACK");
        blinkinLedDriver.close();
    }

    public void reset() {
        setPattern("BLACK");
    }

}
