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
    private RobotBase robotBase;
    private Telemetry telemetry;
    private SampleMecanumDrive mecanumDrive;

    public Pose2d startPose;
    public Trajectory driveToWobbleDrop;
    public Trajectory driveToShoot;
    public Trajectory driveToPark;

    public AutoPath(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
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
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.15, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .addTemporalMarker(0.2, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
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
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.4, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .addTemporalMarker(0.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
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
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.2, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .addTemporalMarker(0.3, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
                        .build();
                break;


                /***************************************************************************

                 RED RIGHT

                 ***************************************************************************/
            case RED_RIGHT_QUAD:
                startPose = new Pose2d(-60,-50,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        //First add the path segments to follow
                        .splineTo(new Vector2d(0.0, -60.0), 0)
                        .splineTo(new Vector2d(42.0, -61.0), 0)
                        //.splineToLinearHeading(new Pose2d(-4.0, -40.0,0),0) //Dont need this as our end direction is sufficient to avoid wobble collision

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(1.5, () -> {
                            //Prep the wobble to drop on the way
                            robotBase.lift.setDropPrepPosition();
                        })
                        .addTemporalMarker(2.0, () -> {
                            //Dump that wobble!
                            robotBase.lift.setDropPosition();
                        })
                        .addTemporalMarker(2.8, () -> {
                            //Dump that wobble!
                            robotBase.lift.liftClaw.open();
                        })
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.15, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .addTemporalMarker(0.2, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
                        .build();
                break;

            case RED_RIGHT_SINGLE:
                startPose = new Pose2d(-60,-50,0.0);
                mecanumDrive.setPoseEstimate(startPose);
                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        //First add the path segments to follow
                        .splineTo(new Vector2d(-24, -52), 0 )
                        .splineTo(new Vector2d(18, -40), 0)
                        //.splineToLinearHeading(new Pose2d(-4.0, -40.0,0),0) //Dont need this, we are driving straight back to shoot.

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(1.0, () -> {
                            //Prep the wobble to drop on the way
                            robotBase.lift.setDropPrepPosition();
                        })
                        .addTemporalMarker(2.0, () -> {
                            //Dump that wobble!
                            robotBase.lift.setDropPosition();
                        })
                        .addTemporalMarker(2.3, () -> {
                            //Dump that wobble!
                            robotBase.lift.liftClaw.open();
                        })
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.4, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .addTemporalMarker(0.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
                        .build();
                break;

            case RED_RIGHT_NONE:
                startPose = new Pose2d(-60.0,-50,0);
                mecanumDrive.setPoseEstimate(startPose);

                driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                        //First add the path segments to follow
                        .splineTo(new Vector2d(-36.0, -60.0), 0)
                        .splineTo(new Vector2d(-6.0, -60.0), 0)
                        //.lineToSplineHeading(new Pose2d(-18.0, -60,0)) //TODO Passes Continuity Test, have to test if its actually better
                        .splineToLinearHeading(new Pose2d(-8.0, -56.0,0),0)

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(1.0, () -> {
                            //Prep the wobble to drop on the way
                            robotBase.lift.setDropPrepPosition();
                        })
                        .addTemporalMarker(2.0, () -> {
                            //Dump that wobble!
                            robotBase.lift.setDropPosition();
                        })
                        .addTemporalMarker(2.1, () -> {
                            //Dump that wobble!
                            robotBase.lift.liftClaw.open();
                        })
                        .build();

                driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                        .strafeTo(new Vector2d(0.0, -40.0))
                        .addTemporalMarker(0.2, () -> {
                            //Start shooter on the way
                            robotBase.shooter.run();
                        })
                        .addTemporalMarker(0.3, () -> {
                            //Retract the lift on the way
                            robotBase.lift.setStorePosition();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                        .strafeTo(new Vector2d(12.0, -40.0))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.shooter.triggerstop();
                            robotBase.shooter.stop();
                        })
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

