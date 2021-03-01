package org.firstinspires.ftc.teamcode.drive.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmodes.AutoPath;

import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.PathName;

@TeleOp(name="AutoPathValidator", group="teleop")
//@Disabled
public class AutoPathValidator extends RobotBase {

    public String pathNameAsString;
    public AutoPath autoPath;
    public SampleMecanumDrive rrdrive;

    /**
     * Constructor
     */
    public AutoPathValidator() {
    }

    @Override
    public void init() {
        autoPath = new AutoPath(hardwareMap,this);
        rrdrive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {

        for (PathName pathName : PathName.values()) {
            pathNameAsString = pathName.toString();
            telemetry.addData("***** Fetching paths for: ", pathNameAsString);
            try {
                autoPath.getAutoPaths(pathName);
            } catch (Exception e) {
                telemetry.addData("Error fetching paths for: ", pathNameAsString);
                telemetry.addData("Exception Message: ",e.toString());
            }
            try {
                rrdrive.setPoseEstimate(autoPath.startPose);
            } catch (Exception e) {
                telemetry.addData("Error setting pose estmiate for: ", pathNameAsString);
                telemetry.addData("Exception Message: ",e.toString());
            }
        }
    }

    @Override
    public void stop() {
    }
}