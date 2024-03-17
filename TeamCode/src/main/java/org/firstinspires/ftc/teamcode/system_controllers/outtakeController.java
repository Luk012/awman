package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLLECT_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_LIFT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;

import java.io.BufferedReader;

@Config
public class outtakeController {

    public enum outtakeStatus
    {
        INITIALIZE,
        SCORE_FOURBAR,
        SCORE_CLAW,
        SCORE_LIFT,
        SCORE_DONE,

        COLLECT_LIFT,
        COLLLECT_CLAW,
        COLLECT_FOURBAR,
        COLLECT_DONE,
    }

    public outtakeController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static outtakeStatus CS = INITIALIZE, PS = INITIALIZE;

    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime lift_timer = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();

    public void update(robotMap r, liftController lift, fourbarController fourbar, clawFlipController clawFlip, clawAngleController clawAngle, doorController door, latchRightController latchRight, latchLeftController latchLeft)
    {
        if(CS != PS || CS == INITIALIZE || CS == SCORE_FOURBAR || CS == SCORE_LIFT || CS == SCORE_CLAW || CS == COLLECT_LIFT || CS == COLLLECT_CLAW || CS == COLLECT_FOURBAR || CS == SCORE_DONE || CS == COLLECT_DONE)
        {
            switch (CS)
            {

                /**
                 * SCORE
                 */

                case SCORE_FOURBAR:
                {
                    fourbar.CS = fourbarController.fourbarStatus.SCORE;
                    lift_timer.reset();
                    CS = SCORE_LIFT;
                    break;
                }

                case SCORE_LIFT:
                {
                    if(lift_timer.seconds() > 0.3)
                    {
                        lift.pid = 1;
                        lift.CS = liftController.liftStatus.UP;
                        claw_timer.reset();
                        CS = SCORE_CLAW;
                    }
                    break;
                }

                case SCORE_CLAW:
                {
                    if(claw_timer.seconds() > 0.05)
                    {
                        clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                    }

                    if(claw_timer.seconds() > 0.1)
                    {
                        clawAngle.clawAngle_i = 2;
                        clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                        CS = SCORE_DONE;
                    }
                    break;
                }

                /**
                 * COLLECT
                 */
                case COLLECT_FOURBAR:
                {
                        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                        claw_timer.reset();
                        CS = COLLLECT_CLAW;
                    break;
                }

                case COLLLECT_CLAW:
                {
                    if(claw_timer.seconds() > 0.1)
                    {
                        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                    }
                    if(claw_timer.seconds() > 0.2)
                    {
                        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                        CS = COLLECT_LIFT;

                    } break;
                }

                case COLLECT_LIFT:
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;

                    lift.pid = 0;
                    lift.CS = liftController.liftStatus.DOWN;
                    claw_timer.reset();
                    CS = COLLECT_DONE;
                    break;
                }




            }
        }

        PS = CS;
    }

}
