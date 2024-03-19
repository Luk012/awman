package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
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
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import java.io.BufferedReader;
import java.util.List;
import java.util.logging.XMLFormatter;

@Config
@Autonomous(group = "Auto" , name = "RedFar")

public class RedFarBun extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE,
        SYSTEMS_PURPLE,
        COLLECT_PURPLE,
        VERIF_TIMER,
        GO_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW_v2,
        VERIF_PURPLE_SCORE,
        YELLOW_DROP,

        PREPARE_COLLECT,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXLES,
        COLLECT_VERIF_PIXLES_V2,

        GO_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE,
        SCORE_CYCLE,
        FAIL_SAFE,
        FAIL_SAFE_WRONG_HEADING,


        NOTHING
    }

    public static double x_start = -43, y_start = -61, angle_start = 270;


    /**
     * purple
     */

    public static double x_purple_preload_right = -44, y_purple_preload_right = -26, angle_purple_preload_right = 185;
    public static double x_purple_preload_center = -57.5, y_purple_preload_center = -23, angle_purple_preload_center = 191;
    public static double x_purple_preload_left = -66, y_purple_preload_left = -26.5, angle_purple_preload_left = 167;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 41, y_yellow_preload_right = -38, angle_yellow_preload_right = 182;
    public static double x_yellow_preload_center = 41, y_yellow_preload_center = -33, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 41, y_yellow_preload_left = -28.5, angle_yellow_preload_left = 180;

    /**
     * collect
     */



    // Cycle 2
    public static double x_inter_collect_cycle_2_right = 30, y_inter_collect_cycle_2_right = -4.2, angle_inter_collect_cycle_2_right = 180;
    public static double x_collect_cycle_2_right = -23, y_collect_cycle_2_right = -4.2, angle_collect_cycle_2_right = 180;

    public static double x_inter_collect_cycle_2_center = 30, y_inter_collect_cycle_2_center = -8, angle_inter_collect_cycle_2_center = 180;
    public static double x_collect_cycle_2_center = -24, y_collect_cycle_2_center = -8, angle_collect_cycle_2_center = 180;

    public static double x_inter_collect_cycle_2_left = 30, y_inter_collect_cycle_2_left = -6, angle_inter_collect_cycle_2_left = 180;
    public static double x_collect_cycle_2_left = -24, y_collect_cycle_2_left = -6, angle_collect_cycle_2_left = 180;

    // Cycle 3
    public static double x_inter_collect_cycle_3_right = 30, y_inter_collect_cycle_3_right = -5, angle_inter_collect_cycle_3_right = 180;
    public static double x_collect_cycle_3_right = -22, y_collect_cycle_3_right = -5, angle_collect_cycle_3_right = 180;

    public static double x_inter_collect_cycle_3_center = 30, y_inter_collect_cycle_3_center = -7.5, angle_inter_collect_cycle_3_center = 180;
    public static double x_collect_cycle_3_center = -22, y_collect_cycle_3_center = -7.5, angle_collect_cycle_3_center = 180;

    public static double x_inter_collect_cycle_3_left = 30, y_inter_collect_cycle_3_left = -6, angle_inter_collect_cycle_3_left = 180;
    public static double x_collect_cycle_3_left = -22, y_collect_cycle_3_left = -6, angle_collect_cycle_3_left = 180;



    /**
     * score
     */


    // Second cycle scoring positions
    public static double x_score_second_cycle_right = 47.5, y_score_second_cycle_right = -7, angle_score_second_angle_right = 150;
    public static double x_score_second_cycle_center = 47.5, y_score_second_cycle_center = -6.5, angle_score_second_angle_center = 150;
    public static double x_score_second_cycle_left = 47.5, y_score_second_cycle_left = -7, angle_score_second_angle_left = 150;

    // Third cycle scoring positions
    public static double x_score_third_cycle_right = 47.5, y_score_third_cycle_right = -6.5, angle_score_third_angle_right = 150;
    public static double x_score_third_cycle_center = 47.5, y_score_third_cycle_center = -6.5, angle_score_third_angle_center = 150;
    public static double x_score_third_cycle_left = 47.5, y_score_third_cycle_left = -6.5, angle_score_third_angle_left = 150;






    /**
     * intern collect
     */

    public static double x_inter_collect_first_cycle = -44, y_inter_collect_first_cycle = -7, angle_inter_collect_first_cycle = 180;

    public static double x_inter_collect_first_cycle_left = -52, y_inter_collect_first_cycle_left = -7, angle_inter_collect_first_cycle_left = 180;

    /**
     * intern score
     */

    public static double x_inter_score_first_cycle = 30, y_inter_score_first_cycle = -8.5, angle_inter_score_first_cycle =180;
    public static double x_inter_score_first_cycle_left = 30, y_inter_score_first_cycle_left = -8.5, angle_inter_score_first_cycle_left =180;

    public static int caz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RedPipelineStackMaster redLeft = new RedPipelineStackMaster(this);
        redLeft.observeStick();




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
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();

        RedFarAutoController redFarAutoController = new RedFarAutoController();


        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.CLOSED;
        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
        lift.CS = liftController.liftStatus.INITIALIZE;
        extendo.CS = extendoController.extendoStatus.RETRACTED;

//
        drive.update();
        clawAngle.update(r);
        clawFlip.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

        collectAngle.update(r);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d interCollectFirstCycle = new Pose2d(x_inter_collect_first_cycle, y_inter_collect_first_cycle, Math.toRadians(angle_inter_collect_first_cycle));
        Pose2d interScoreFirstCycle = new Pose2d(x_inter_score_first_cycle, y_inter_score_first_cycle, Math.toRadians(angle_inter_score_first_cycle));

        Pose2d interCollectFirstCycleleft = new Pose2d(x_inter_collect_first_cycle_left, y_inter_collect_first_cycle_left, Math.toRadians(angle_inter_collect_first_cycle_left));
        Pose2d interScoreFirstCycleleft = new Pose2d(x_inter_score_first_cycle_left, y_inter_score_first_cycle_left, Math.toRadians(angle_inter_score_first_cycle_left));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));

        // Cycle 2 Right
        Pose2d collect_inter_cycle2_right = new Pose2d(x_inter_collect_cycle_2_right, y_inter_collect_cycle_2_right, Math.toRadians(angle_inter_collect_cycle_2_right));
        Pose2d collect_cycle2_right = new Pose2d(x_collect_cycle_2_right, y_collect_cycle_2_right, Math.toRadians(angle_collect_cycle_2_right));

// Cycle 2 Center
        Pose2d collect_inter_cycle2_center = new Pose2d(x_inter_collect_cycle_2_center, y_inter_collect_cycle_2_center, Math.toRadians(angle_inter_collect_cycle_2_center));
        Pose2d collect_cycle2_center = new Pose2d(x_collect_cycle_2_center, y_collect_cycle_2_center, Math.toRadians(angle_collect_cycle_2_center));

// Cycle 2 Left
        Pose2d collect_inter_cycle2_left = new Pose2d(x_inter_collect_cycle_2_left, y_inter_collect_cycle_2_left, Math.toRadians(angle_inter_collect_cycle_2_left));
        Pose2d collect_cycle2_left = new Pose2d(x_collect_cycle_2_left, y_collect_cycle_2_left, Math.toRadians(angle_collect_cycle_2_left));

// Cycle 3 Right
        Pose2d collect_inter_cycle3_right = new Pose2d(x_inter_collect_cycle_3_right, y_inter_collect_cycle_3_right, Math.toRadians(angle_inter_collect_cycle_3_right));
        Pose2d collect_cycle3_right = new Pose2d(x_collect_cycle_3_right, y_collect_cycle_3_right, Math.toRadians(angle_collect_cycle_3_right));

// Cycle 3 Center
        Pose2d collect_inter_cycle3_center = new Pose2d(x_inter_collect_cycle_3_center, y_inter_collect_cycle_3_center, Math.toRadians(angle_inter_collect_cycle_3_center));
        Pose2d collect_cycle3_center = new Pose2d(x_collect_cycle_3_center, y_collect_cycle_3_center, Math.toRadians(angle_collect_cycle_3_center));

// Cycle 3 Left
        Pose2d collect_inter_cycle3_left = new Pose2d(x_inter_collect_cycle_3_left, y_inter_collect_cycle_3_left, Math.toRadians(angle_inter_collect_cycle_3_left));
        Pose2d collect_cycle3_left = new Pose2d(x_collect_cycle_3_left, y_collect_cycle_3_left, Math.toRadians(angle_collect_cycle_3_left));



        // Second cycle scoring positions
        Pose2d score_second_cycle_right = new Pose2d(x_score_second_cycle_right, y_score_second_cycle_right, Math.toRadians(angle_score_second_angle_right));
        Pose2d score_second_cycle_center = new Pose2d(x_score_second_cycle_center, y_score_second_cycle_center, Math.toRadians(angle_score_second_angle_center));
        Pose2d score_second_cycle_left = new Pose2d(x_score_second_cycle_left, y_score_second_cycle_left, Math.toRadians(angle_score_second_angle_left));

// Third cycle scoring positions
        Pose2d score_third_cycle_right = new Pose2d(x_score_third_cycle_right, y_score_third_cycle_right, Math.toRadians(angle_score_third_angle_right));
        Pose2d score_third_cycle_center = new Pose2d(x_score_third_cycle_center, y_score_third_cycle_center, Math.toRadians(angle_score_third_angle_center));
        Pose2d score_third_cycle_left = new Pose2d(x_score_third_cycle_left, y_score_third_cycle_left, Math.toRadians(angle_score_third_angle_left));




        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleLeft)
                .build();
        
        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
                .lineToLinearHeading(interCollectFirstCycleleft)
                .lineToLinearHeading(interScoreFirstCycleleft)
                .lineToLinearHeading(yellowLeft)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
                .lineToLinearHeading(interCollectFirstCycle)
                .lineToLinearHeading(interScoreFirstCycle)
                .lineToLinearHeading(yellowCenter)
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(interCollectFirstCycle)
                .lineToLinearHeading(interScoreFirstCycle)
                .lineToLinearHeading(yellowRight)
                .build();


        TrajectorySequence YELLOW_DROP_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                        .back(12)
                                .build();

        TrajectorySequence YELLOW_DROP_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .back(12)
                .build();


        TrajectorySequence YELLOW_DROP_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .back(12)
                .build();

        TrajectorySequence COLLECT_CYCLE_2_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                .lineToLinearHeading(collect_inter_cycle2_right)
                .lineToLinearHeading(collect_cycle2_right)
                .build();

        TrajectorySequence COLLECT_CYCLE_2_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .lineToLinearHeading(collect_inter_cycle2_center)
                .lineToLinearHeading(collect_cycle2_center)
                .build();

        TrajectorySequence COLLECT_CYCLE_2_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .lineToLinearHeading(collect_inter_cycle2_left)
                .lineToLinearHeading(collect_cycle2_left)
                .build();

        TrajectorySequence COLLECT_CYCLE_3_RIGHT = drive.trajectorySequenceBuilder(score_second_cycle_right)
                .lineToLinearHeading(collect_inter_cycle3_right)
                .lineToLinearHeading(collect_cycle3_right)
                .build();

        TrajectorySequence COLLECT_CYCLE_3_CENTER = drive.trajectorySequenceBuilder(score_second_cycle_center)
                .lineToLinearHeading(collect_inter_cycle3_center)
                .lineToLinearHeading(collect_cycle3_center)
                .build();

        TrajectorySequence COLLECT_CYCLE_3_LEFT = drive.trajectorySequenceBuilder(score_second_cycle_left)
                .lineToLinearHeading(collect_inter_cycle3_left)
                .lineToLinearHeading(collect_cycle3_left)
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_RIGHT = drive.trajectorySequenceBuilder(collect_cycle2_right)
                .lineToLinearHeading(score_second_cycle_right)
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_CENTER = drive.trajectorySequenceBuilder(collect_cycle2_center)
                .lineToLinearHeading(score_second_cycle_center)
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_LEFT = drive.trajectorySequenceBuilder(collect_cycle2_left)
                .lineToLinearHeading(score_second_cycle_left)
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_RIGHT = drive.trajectorySequenceBuilder(collect_cycle3_right)
                .lineToLinearHeading(score_third_cycle_right)
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_CENTER = drive.trajectorySequenceBuilder(collect_cycle3_center)
                .lineToLinearHeading(score_third_cycle_center)
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_LEFT = drive.trajectorySequenceBuilder(collect_cycle3_left)
                .lineToLinearHeading(score_third_cycle_left)
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;


        int nrcicluri = 0;
        double loopTime = 0;
        double extendo_timer_i[] = {3.25, 2.8, 3};

        double extendo_timer_i_purple[] = {1.8, 1.7, 1.4};

        ElapsedTime transfer = new ElapsedTime();
        ElapsedTime prepare_score_yellowqe = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime verif_pixels = new ElapsedTime();
                ElapsedTime preload = new ElapsedTime();
        ElapsedTime verif = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime failsafe = new ElapsedTime();

        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;


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


            MotorConfigurationType motorConfigurationType = r.extendoRight.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoRight.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendoLeft.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoLeft.setMotorType(motorConfigurationType);

            motorConfigurationType = r.lift.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.lift.setMotorType(motorConfigurationType);

            //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);

            switch (status) {

                case START: {

                    switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                            extendo.caz = 2;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            extendo.caz = 1;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                            extendo.caz = 0;
                                break;
                        }
                    }

                    preload.reset();
                    status = STROBOT.SYSTEMS_PURPLE;
                    break;
                }

                case SYSTEMS_PURPLE:
                {
                    if(preload.seconds() > 0.1)
                    {
                        redLeft.stopCamera();
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE;
                        extendo_timer.reset();
                        status = STROBOT.PURPLE;
                    }
                    break;
                }

                case PURPLE:
                {

                    if(extendo_timer.seconds() > extendo_timer_i_purple[caz])
                    {
                        r.collect.setPower(1);
                        extendo.CS = extendoController.extendoStatus.PURPLE;
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case COLLECT_PURPLE:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {

                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        verif.reset();
                      status = STROBOT.GO_SCORE_YELLOW;
                        }
                    break;

                }

                case GO_SCORE_YELLOW:
                {

                    switch (caz)
                    {
                        case 0:

                        {
                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-0.5);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-0.5);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-0.5);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                    }

                    break;

                }

                case PREPARE_SCORE_YELLOW:
                {

                    if(transfer.seconds() > 0.5)
                    {
                        r.collect.setPower(0);
                    }

                    if(transfer.seconds() > 1.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        prepare_score_yellowqe.reset();
                        status = STROBOT.PREPARE_SCORE_YELLOW_v2;
                    }
                    break;
                }

                case PREPARE_SCORE_YELLOW_v2:
                {
                    if(drive.getPoseEstimate().getX() >= 15 && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN;
                        status = STROBOT.VERIF_PURPLE_SCORE;
                    }

                    break;
                }

                case VERIF_PURPLE_SCORE:
                {
                    if(!drive.isBusy() && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE)
                    {
                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_LEFT);
                                break;
                            }

                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_CENTER);
                                break;
                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_DROP_RIGHT);
                                break;
                            }
                        }

                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
                    if(r.back.getDistance(DistanceUnit.CM) < 20.8)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_LEFT);
                                break;
                            }

                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_CENTER);
                                break;
                            }

                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_RIGHT);
                                break;
                            }
                        }
                        prepare_collect.reset();
                        status= STROBOT.PREPARE_COLLECT;
                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(prepare_collect.seconds() > 0.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                    }
                    break;
                }

                case COLLECT_EXTENDO:
                {
                    if(extendo_timer.seconds() > extendo_timer_i[nrcicluri])
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES:
                {
                    if(!r.pixelLeft.getState() || !r.pixelRight.getState())
                    {
                             collectAngle.collectAngle_i -=1;
                              status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    } else if(failsafe.seconds() > 1.6 && (r.pixelLeft.getState() && r.pixelRight.getState()))
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.FAIL_SAFE;
                        status = STROBOT.FAIL_SAFE;
                    }
                    break;
                }

                case FAIL_SAFE:
                {
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.FAIL_SAFE_DONE)
                    {
                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    } else if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.FAIL_SAFE_WRONG_HEADING)
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        r.collect.setPower(0);
                        failsafe.reset();
                        status = STROBOT.FAIL_SAFE_WRONG_HEADING;
                    }
                    break;
                }

                case FAIL_SAFE_WRONG_HEADING:
                {
                    if(failsafe.seconds() > 1.5)
                    {   collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                    r.collect.setPower(1);

                    switch (nrcicluri)
                    {
                        case 0:
                        {
                            collectAngle.collectAngle_i = 4;
                            extendo.CS = extendoController.extendoStatus.CYCLE;
                            failsafe.reset();
                            break;
                        }
                        case 1:
                        {
                            collectAngle.collectAngle_i = 2;
                            extendo.CS = extendoController.extendoStatus.CYCLE;
                            failsafe.reset();
                            break;
                        }
                        case 2:
                        {
                            collectAngle.collectAngle_i = 0;
                            extendo.CS = extendoController.extendoStatus.CYCLE;
                            failsafe.reset();
                            break;
                        }

                    }
                    status = STROBOT.COLLECT_VERIF_PIXLES;}
                    break;
                }



                case COLLECT_VERIF_PIXLES_V2:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        extendo_timer.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case GO_SCORE_CYCLE:
                {
                    if(extendo_timer.seconds() > 0.2)
                    { r.collect.setPower(0);}
                    if(extendo_timer.seconds() > 0.3)
                    {
                        r.collect.setPower(-1);
                        status = STROBOT.PREPARE_SCORE_CYCLE;
                    }
                    switch (nrcicluri) {
                        case 0:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_RIGHT);
                                    break;
                            }
                            break;

                        case 1:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_RIGHT);
                                    break;
                            }
                            break;


                    }
                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;

                    break;
                }

                case PREPARE_SCORE_CYCLE:
                {r.collect.setPower(0);
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri +=1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_CYCLE_DONE && score.seconds() > 0.75)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                        switch (nrcicluri) {
                            case 1:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_RIGHT);
                                        break;


                                }
                                break;

                                default:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_LEFT);
                                    break;

                        }
                        prepare_collect.reset();
                        status= STROBOT.PREPARE_COLLECT;
                    }

                    break;
                }


            }

            drive.update();
            clawAngle.update(r);
            clawFlip.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            drone.update(r);
            lift.update(r, position, voltage);
            extendo.update(r, extendopos, 1, voltage);
            collectAngle.update(r);
            redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);



//            telemetry.addData("status", status);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("flip", clawFlip.CS);
//            telemetry.addData("angle", clawAngle.CS);
//            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);

           // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
            telemetry.addData("robotcontroller", RedFarAutoController.CurrentStatus);
            telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
          //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
