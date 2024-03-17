package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Boolean.FALSE;

import android.sax.TextElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPixelMaster;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchDropController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.ptoController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedReader;
import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "BlueLeftNearWall")

public class RedFar extends LinearOpMode {

    enum STROBOT {
        START,
        SYSTEMS,
        NOTHING,
        CHECK_COLLECT,
        CHECK_COLLECT_V2,
        REVERSE,
        INTERN,
        DETECT_YELLO,
        DETECT_APRIL_TAG,
        YELLOW_LEFT,
        YELLOW_RIGHT,

    }

    public static double x_start = -43, y_start = -61, angle_start = 270;

    private static final int DESIRED_TAG_ID = 6;




    /**
     * purple
     */

    public static double x_purple_preload_right = -42.5, y_purple_preload_right = -30, angle_purple_preload_right = 170;
    public static double x_purple_preload_center = -54, y_purple_preload_center = -26, angle_purple_preload_center = 177;
    public static double x_purple_preload_left = -62, y_purple_preload_left = -26, angle_purple_preload_left = 170;

    /**
     * collect
     */

    public static double x_collect = -19, y_collect = -7, angle_collect = 180;

    public static double x_collect_middle = -20, y_collect_middle = -6.5, angle_collect_middle = 195;

    /**
     * score
     */


    public static double x_score = 47, y_score = -10, angle_score = 150;

    public static double apriltag_x = 41, apriltag_y= -33.5, apriltag_angle = 180;

    /**
     * interns
     */

    public static double x_inter = -36, y_inter=-10,angle_intern =180;
    public static double x_inter2 = 30, y_inter2=-10,angle_intern2 =180;

    int caz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RedPipelineStackMaster redLeft = new RedPipelineStackMaster(this);
        redLeft.observeStick();

        YellowPixelMaster yellowPixel = new YellowPixelMaster(this);
        yellowPixel.observeStick();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap r = new robotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        collectAngleController collectAngle = new collectAngleController();
        doorController door = new doorController();
        fourbarController fourbar = new fourbarController();
        latchLeftController latchLeft = new latchLeftController();
        latchRightController latchRight = new latchRightController();
        ptoController pto = new ptoController();
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();
        transferController transfer = new transferController();
        outtakeController outtake = new outtakeController();
        latchDropController latchDrop = new latchDropController();

        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.INITIALIZE;
        clawFlip.CS = clawFlipController.clawFlipStatus.INITIALIZE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.INITIALIZE;
        fourbar.CS = fourbarController.fourbarStatus.INITIALIZE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
        pto.CS = ptoController.ptoStatus.INITIALIZE;
        lift.CS = liftController.liftStatus.INITIALIZE;
        extendo.CS = extendoController.extendoStatus.INITIALIZE;

//
        drive.update();
        clawAngle.update(r);
        clawFlip.update(r);
        clawAngle.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        pto.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo);
        outtake.update(r, lift, fourbar, clawFlip, clawAngle, door, latchRight, latchLeft);
        latchDrop.update(r, latchRight, latchLeft, clawAngle);
        collectAngle.update(r);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d intern = new Pose2d(x_inter, y_inter,Math.toRadians(angle_intern));
        Pose2d intern2 = new Pose2d(x_inter2, y_inter2,Math.toRadians(angle_intern2));
        Pose2d apriltag = new Pose2d(apriltag_x, apriltag_y, Math.toRadians(apriltag_angle));

        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleLeft)
                .build();

        TrajectorySequence INTERN = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(intern)
                .lineToLinearHeading(intern2)
                .lineToLinearHeading(apriltag)
                .build();

        TrajectorySequence YELLOW = drive.trajectorySequenceBuilder(apriltag)
                .back(8,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence FRONT = drive.trajectorySequenceBuilder(apriltag)
                        .forward(5)
                                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

ElapsedTime extendo_timer = new ElapsedTime();
ElapsedTime pixel_timer = new ElapsedTime();
ElapsedTime reverse = new ElapsedTime();
        double nrcicluri = 0;
        double loopTime = 0;
//        extendo.ciclu = 0;
//       //extendo.target = 120;
//        extendo.target = 280;
//        extendo.difference = 650;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;

        //String yellowCase = null;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(redLeft.opencvred.getWhichSide() == "left"){
                caz = 0;
            } else if (redLeft.opencvred.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", redLeft.opencvred.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();
            int extendopos = r.extendoLeft.getCurrentPosition();

          //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);

            switch (status) {

                case START: {
                    drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                    extendo_timer.reset();
lift.CS = liftController.liftStatus.DOWN;
                    status = STROBOT.SYSTEMS;
                    break;
                }

                case SYSTEMS:
                {

                    fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                    clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;

                    if(!drive.isBusy())
                    {
                        r.collect.setPower(1);
                        extendo.CS = extendoController.extendoStatus.PURPLE;
                        latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                        latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                        pixel_timer.reset();
                        status = STROBOT.CHECK_COLLECT;
                    }
                  break;
                }

                case CHECK_COLLECT:
                {
                    if(pixel_timer.seconds() > 0.2) {
                        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        status = STROBOT.CHECK_COLLECT_V2;
                    }

                    break;
                }

                case CHECK_COLLECT_V2:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                       // r.collect.setPower(0);
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        reverse.reset();
                        status = STROBOT.REVERSE;

                    }
                    break;
                }

                case REVERSE:
                {
                    if(reverse.seconds() > 0.25)
                    {
                        r.collect.setPower(-0.5);
                        extendo_timer.reset();
                        status = STROBOT.INTERN;
                    }
                    break;
                }

                case INTERN:
                {
                    drive.followTrajectorySequenceAsync(INTERN);
                    if(extendo_timer.seconds() > 0.5)
                    { r.collect.setPower(0);}
                    if(extendo_timer.seconds() > 1.2)
                    {transfer.CS = transferController.transferStatus.TRANSFER_EXTENDO;
                        pixel_timer.reset();
                    status = STROBOT.DETECT_YELLO;}
                    break;
                }

                case DETECT_YELLO:
                {
                    if(pixel_timer.seconds() > 4 && transfer.CS == transferController.transferStatus.TRANSFER_DONE)
                    {
                        outtake.CS = outtakeController.outtakeStatus.SCORE_FOURBAR;
                    }
                    if(!drive.isBusy())
                    {

                        if(yellowPixel.yellowPixel.getWhichSide() == "left")
                        {
                            clawAngle.clawAngle_i = 1;
                            drive.followTrajectorySequenceAsync(YELLOW);
                            status = STROBOT.YELLOW_LEFT;
                        } else
                        {
                            clawAngle.clawAngle_i = 3;
                            drive.followTrajectorySequenceAsync(YELLOW);
                            status = STROBOT.YELLOW_RIGHT;
                        }
                    }
                    break;
                }

                case YELLOW_LEFT:
                {
                    if(r.back.getDistance(DistanceUnit.CM) < 22.5)
                    {
                        latchDrop.CS = latchDropController.latchDropStatus.DROP_BOTH;
                        drive.followTrajectorySequenceAsync(FRONT);
                        status = STROBOT.NOTHING;
                    }
                    break;
                }

                case YELLOW_RIGHT:
                {
                    if(r.back.getDistance(DistanceUnit.CM) < 22)
                    {
                        latchDrop.CS = latchDropController.latchDropStatus.DROP_BOTH;
                        drive.followTrajectorySequenceAsync(FRONT);
                        status = STROBOT.NOTHING;
                    }
                    break;
                }




            }

            drive.update();
            clawAngle.update(r);
            clawFlip.update(r);
            clawAngle.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            pto.update(r);
            drone.update(r);
            lift.update(r, position, voltage);
            extendo.update(r, extendopos, 1, voltage);
            transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo);
            outtake.update(r, lift, fourbar, clawFlip, clawAngle, door, latchRight, latchLeft);
            latchDrop.update(r, latchRight, latchLeft, clawAngle);
            collectAngle.update(r);

            telemetry.addData("yellowCase", yellowPixel.yellowPixel.getWhichSide());
            telemetry.addData("status", status);
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
           telemetry.addData("position", extendopos);
         //   telemetry.addData("target", extendo.target);
        //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
