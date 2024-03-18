package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class liftController {

    public enum liftStatus
    {
        INITIALIZE,
        DOWN,
        UP,
        CYCLE,

        PRELOAD_YELLOW,
    }

    public double CurrentSpeed = 0;

    /**
     * UP
     */

//    public static double P1 = 0.03;
//    public static double I1 = 0.0031;
//    public static double D1 = 0;

    public static double P1 = 0.0045;
    public static double I1 = 0.0012;
    public static double D1 = 0.001;

//    public static double P1 = 0.0075;
//    public static double I1 = 0.0012;
//    public static double D1 = 0.001;

    public static double P2 = 0.03;
    public static double I2 = 0.0031;
    public static double D2 = 0;

    public double pid = 0;

    public double Kg = 0;
    public double maxSpeedUp = 1;

    public static liftStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController LiftPIDUP = null;
    SimplePIDController LiftPIDDOWN = null;


    int base = 0;

    public static double down = 108;

    public static int i_up = 0;
    public static double i_multiplication = 108;

    public int CurrentPosition = 0;

    public liftController()
    {
        LiftPIDUP = new SimplePIDController(P1,I1,D1);
        LiftPIDUP.targetValue = base;
        LiftPIDUP.maxOutput = maxSpeedUp;

        LiftPIDDOWN = new SimplePIDController(P2,I2,D2);
        LiftPIDDOWN.targetValue = base;
        LiftPIDDOWN.maxOutput = maxSpeedUp;
    }

    public void update(robotMap r, int position, double voltage)
    {

        CurrentPosition = position;

        if(CS == liftStatus.UP) LiftPIDUP.targetValue = down + i_up * i_multiplication;



        double powerLiftUp = LiftPIDUP.update(position) + Kg;
        powerLiftUp = Math.max(-1,Math.min(powerLiftUp* 14 / voltage,1));
        CurrentSpeed=powerLiftUp;

        double powerLiftDown = LiftPIDDOWN.update(position) + Kg;
        powerLiftDown = Math.max(-1,Math.min(powerLiftDown* 14 / voltage,1));
        CurrentSpeed=powerLiftDown;

        double powerLiftFinal;

        powerLiftFinal = (pid == 0) ? powerLiftDown : powerLiftUp;

        r.lift.setPower(powerLiftFinal);

        if(CS != PS || CS == INITIALIZE )
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    LiftPIDUP.targetValue = -10;
                    break;
                }

                case UP:
                {
                    LiftPIDUP.targetValue = down + i_up * i_multiplication;
                    break;
                }

                case DOWN:
                {
                    LiftPIDDOWN.targetValue = -10;
                    break;
                }

                case PRELOAD_YELLOW:
                {
                    LiftPIDUP.targetValue = 300;
                    break;
                }

                case CYCLE:
                {
                    LiftPIDUP.targetValue = 370;
                    break;
                }
            }
        }

        PS = CS;
    }

}
