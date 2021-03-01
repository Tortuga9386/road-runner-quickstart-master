package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.PathName;

public class AutoPath {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private SampleMecanumDrive mecanumDrive;
    public Pose2d startPose;
    public Trajectory driveToWobbleDrop;
    public Trajectory driveToShoot;
    public Trajectory driveToPark;

    public AutoPath(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
    }
    public void getAutoPaths(PathName pathName) {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        /*
            All these cases should be handled!
            BLUE_LEFT_QUAD,BLUE_LEFT_SINGLE,BLUE_LEFT_NONE,
            BLUE_RIGHT_QUAD,BLUE_RIGHT_SINGLE,BLUE_RIGHT_NONE,
            RED_LEFT_QUAD,RED_LEFT_SINGLE,RED_LEFT_NONE,
            RED_RIGHT_QUAD,RED_RIGHT_SINGLE,RED_RIGHT_NONE
        */
        switch (pathName){

            /***************************************************************************

             BLUE LEFT

             ***************************************************************************/
            case BLUE_LEFT_QUAD:
                startPose = new Pose2d(-60.0,50.0,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-24.0, 60.0), 0)
                        .lineTo(new Vector2d(48.0, 60.0))
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .lineTo(new Vector2d(-3.0, 36.0))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12.0, 36.0))
                        .build();
                break;

            case BLUE_LEFT_SINGLE:
                startPose = new Pose2d(-60.0,48.0,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-36.0, 60.0), 0)
                        .splineTo(new Vector2d(24.00, 36.00), 0)
                        .build();

                //TODO
                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .lineTo(new Vector2d(-3.0, 36.0))
                        .build();

                //TODO
                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12.0, 36.0))
                        .build();
                break;

            case BLUE_LEFT_NONE:
                startPose = new Pose2d(-60.0,48.0,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-36, 60), 0)
                        .lineTo(new Vector2d(0, 60.0))
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(-3, 35), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12, 35))
                        .build();
                break;

                /***************************************************************************

                 BLUE RIGHT

                 ***************************************************************************/
            case BLUE_RIGHT_QUAD:
                startPose = new Pose2d(-60.0,25,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-24, 60.0), 0)
                        .lineTo(new Vector2d(50.0, 60.0))
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(-3, 36.0), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12, 36.0))
                        .build();
                break;

            case BLUE_RIGHT_SINGLE:
                startPose = new Pose2d(-60.0,25,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-20, 22.0), 0)
                        .splineTo(new Vector2d(24.0, 36.0), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(-3, 32.0), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(10, 32.0))
                        .build();
                break;

            case BLUE_RIGHT_NONE:
                startPose = new Pose2d(-60.0,26,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-24, 60.0), 0)
                        .splineTo(new Vector2d(0.0, 55.0), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(-3, 35.0), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12.0, 35.0))
                        .build();
                break;

                /***************************************************************************

                 RED LEFT

                 ***************************************************************************/
            case RED_LEFT_QUAD:
                startPose = new Pose2d(-60.0,-20,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(48.0, -20.0), 0)
                        .splineTo(new Vector2d(60.0, -50.0), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(0.0, -12.0), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(12.0, -12.0))
                        .build();
                break;

            case RED_LEFT_SINGLE:
                startPose = new Pose2d(-60.0,-26,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-18.0, -23.0), 0)
                        .splineTo(new Vector2d(24.0, -36.0), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d(1.0, -10.0), 0)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(10, -10.0))
                        .build();
                break;

            case RED_LEFT_NONE:
                startPose = new Pose2d(-60.0,-26,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(0.0, -18.0), 0)
                        .splineTo(new Vector2d(11.0, -50.0), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .splineTo(new Vector2d( 1,-10 ), 270)
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(10.0, -10.0))
                        .build();
                break;

                /***************************************************************************

                 RED RIGHT

                 ***************************************************************************/
            case RED_RIGHT_NONE:
                startPose = new Pose2d(-60.0,-50,0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-36.0, -60.0), 0)
                        .splineTo(new Vector2d(0.0, -60.0), 0)
                        .build();

                // Works with no shoot
//                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
//                        .strafeTo(new Vector2d(-18.0, -60))
                        //.build();

                        driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                                .strafeTo(new Vector2d(-18.0, -60))
                                .splineToConstantHeading(new Vector2d(-18.0, -60),  Math.toRadians(0))
                        //.splineTo(new Vector2d(-12.0, -60.0), Math.toRadians(0))
                        //.splineTo(new Vector2d(-12.0, -56.0), Math.toRadians(90))
                        //.splineTo(new Vector2d(-1.0, -12.0), Math.toRadians(0))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12, -12.0))
                        .build();
                break;

            case RED_RIGHT_SINGLE:
                startPose = new Pose2d(-60,-50,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-24, -60), 0 )
                        .splineTo(new Vector2d(23, -40), 0)
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .strafeTo(new Vector2d(-18, -40))
                        .splineToConstantHeading(new Vector2d(-5.0, -40.0), Math.toRadians(0))
                        //.splineTo(new Vector2d(-1.0, -12.0), Math.toRadians(0))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12, -12.0))
                        .build();
                break;

            case RED_RIGHT_QUAD:
                startPose = new Pose2d(-60,-50,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(0, -60), 0)
                        .lineTo(new Vector2d(45,-60))
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .strafeTo(new Vector2d(-18, -60))
                        //.splineTo(new Vector2d(12, -60), Math.toRadians(90))
                        //.splineTo(new Vector2d(-1,-12), Math.toRadians(0))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12, -12.0))
                        .build();
                break;

            /***************************************************************************

             DEFAULT

             ***************************************************************************/
            default:
                telemetry.addData("DEFAULT paths for: ", pathName.toString());
                //If we get here, go nowhere to prevent the robot from hurting itself
                startPose = new Pose2d();
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(1, 1))
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .lineTo(new Vector2d(-1, -1))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .lineTo(new Vector2d(1, 1))
                        .build();
                break;

        }

    }

}

