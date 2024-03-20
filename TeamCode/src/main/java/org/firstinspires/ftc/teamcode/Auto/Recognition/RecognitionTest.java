package org.firstinspires.ftc.teamcode.Auto.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;


@Autonomous(name = "Recognition test")
public class RecognitionTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        OpenCVMaster yellowTest = new OpenCVMaster(this);
        //   ConceptAprilTag test = new ConceptAprilTag();

        yellowTest.observeStick();

        telemetry.addData("Item: ", yellowTest.opencv.getWhichSide());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //  test.runOpMode();
            telemetry.addData("Item: ", yellowTest.opencv.getWhichSide());
            //telemetry.addData("imx", yellowTest.opencv.imx);
//            telemetry.addData("avg_leftLeft",yellowTest.yellowPixel.avg_left);
//            telemetry.addData("avg_leftRight",yellowTest.yellowPixel.avg_right);

            telemetry.update();
            sleep(100);
        }

        yellowTest.stopCamera();

    }
}
