package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.PURPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class extendoController {

    public enum extendoStatus
    {
        INITIALIZE,
        RETRACTED,
        EXTENDED,
        PURPLE,
        CYCLE,
        DRIVE,
    }

    public static double Kp = 0.0041;
    public static double Ki = 0.0033;
    public static double Kd = 0.0033;

    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController extendoPID = null;

    public static double CurrentPosition = 0;
    public static double retracted = -7;
    public static double extended = 900;
    public static double drive = 600;
    public static double purple[] = {375, 160, 375};
    public static double cycle[] = {920, 920, 920};
    public static int cycle_i = 0;
    public static int caz = 0;


    public static double extend_multiply_index = 0;

      public extendoController()
      {
          extendoPID = new SimplePIDController(Kp, Ki, Kd);
          extendoPID.targetValue = retracted;
          extendoPID.maxOutput = maxSpeed;
      }

      public void update(robotMap r, int position, double powerCap, double voltage)
      {

          CurrentPosition = position;
          double powerColectare = extendoPID.update(position);
          powerColectare =  Math.max(-powerCap,Math.min(powerColectare* 14 / voltage,powerCap));
         r.extendoLeft.setPower(powerColectare);
          r.extendoRight.setPower(powerColectare);

          if(CS == EXTENDED)
      {
          extendoPID.targetValue = extended + extend_multiply_index;
      }

//          if(CS == SENSOR && CurrentPosition >= auto[ciclu] - 10)
//          {
//              extendoPID.targetValue = difference;
//              extendoPID.maxOutput = 0.6;
//          }

         if(CS != PS || CS == INITIALIZE || CS == EXTENDED || CS == RETRACTED || CS == PURPLE )
         {
             switch (CS)
             {
                 case INITIALIZE:
                 {
                     extendoPID.targetValue = retracted;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case EXTENDED:
                 {
                     extendoPID.targetValue = extended + extend_multiply_index;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case RETRACTED:
                 {
                     extendoPID.targetValue = retracted;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case PURPLE:
                 {
                     extendoPID.targetValue = purple[caz];
                     extendoPID.maxOutput = 0.75;
                     //CS = SENSOR;
                     break;
                 }

                 case CYCLE:
                 {
                     extendoPID.targetValue = cycle[cycle_i];
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case DRIVE:
                 {
                     extendoPID.targetValue = drive;
                     extendoPID.maxOutput = 1;
                     break;
                 }




             }

             PS = CS;
         }

      }

}
