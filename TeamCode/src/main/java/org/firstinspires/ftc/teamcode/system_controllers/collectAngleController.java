package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class collectAngleController {

    public enum collectAngleStatus
    {
        INITIALIZE,
        COLLECT,
        DRIVE,
    }

    public collectAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static collectAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect[] = {0.75 , 0.71, 0.67, 0.63, 0.59};
    public static double drive = 0.4;
    public static double init = 0.3;


    public static int collectAngle_i = 0;

    public void update(robotMap r)
    {

        if(CS == COLLECT)
        {
            r.collectAngle.setPosition(collect[collectAngle_i]);
        }

         if(CS != PS || CS == INITIALIZE)
         {
             switch (CS)
             {
                 case INITIALIZE:
                 {
                     r.collectAngle.setPosition(init);
                     break;
                 }

                 case COLLECT:
                 {
                     r.collectAngle.setPosition(collect[collectAngle_i]);
                     break;
                 }

                 case DRIVE:
                 {
                     r.collectAngle.setPosition(drive);
                 }
             }
         }

         PS = CS;
    }

}
