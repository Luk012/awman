package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.CLOSED;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.OPENED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class doorController {

    public enum doorStatus
    {
        INITIALIZE,
        OPENED,
        CLOSED,
    }

    public doorController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static doorStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double opened = 0.09;
    public static double closed = 0.76;

    public void update(robotMap r)
    {
        if(CS != PS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.door.setPosition(closed);
                    break;
                }

                case OPENED:
                {
                    r.door.setPosition(opened);
                    break;
                }

                case CLOSED:
                {
                    r.door.setPosition(closed);
                    break;
                }
            }
        }

        PS = CS;
    }
}
