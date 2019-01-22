package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;

import java.util.List;

@Autonomous(name="Auton Tensorflow Depot", group="Minibot")
public class AutonTensorFlowDepot extends LinearOpMode {
    public Variables Variables = new Variables();
    public Methods Methods = new Methods();
    public Hardware Hardware = new Hardware();
    public void runOpMode() {
        Methods.initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            telemetry.addData("1", null);
            telemetry.update();
            Methods.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addLine("Press play to start.");
        telemetry.update();
        waitForStart();
        Variables.runtime.reset();
        Methods.elevatorDrive(1, 9, 4);
        Methods.encoderDrive(Variables.DRIVE_SPEED, 100, 100, 4);
        Methods.elevatorDrive(1, -9, 4);
        Methods.encoderDrive(Variables.DRIVE_SPEED, -100, -100, 4);
        //initVuforia();
       /*try
        {
            initVuforia();
        }
        catch (Exception e)‏
        {
            telemetry.addData("ERROR)", e);
        }
*/

        /** Wait for the game to begin */
        if (Variables.tfod != null) {
            Variables.tfod.activate();
        }
        boolean tFodDone = false;
        while (opModeIsActive() && (tFodDone == false)) {
            if (Variables.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                final int fov = 78;
                final float d_per_pix = (float) 0.040625;
                Methods.gyroTurnTo(Variables.TURN_SPEED, 225);
                sleep(500);
                List<Recognition> updatedRecognitions = Variables.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    sleep(1500);
                    updatedRecognitions = Variables.tfod.getUpdatedRecognitions();
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(Variables.LABEL_GOLD_MINERAL)) {
                            Hardware. leftDrive.setPower(0);
                            Hardware.rightDrive.setPower(0);
                            goldMineralX = (int) recognition.getLeft();
                            telemetry.addLine("Skipper we did it, we found the gold mineral");
                            telemetry.addData("Gold getLeft Value: ", recognition.getLeft());
                            telemetry.addData("Gold getRight Value: ", recognition.getRight());
                            float getleftval = (int) recognition.getLeft();
                            float getrightval = (int) recognition.getRight();
                            float p = (float) (0.5 * (recognition.getLeft() + recognition.getRight()));
                            telemetry.addData("Gold p Value that was calculated: ", p);
                            float h = (float) (p - (0.5 * 800));
                            telemetry.addData("h value calculated ", h);
                            float angle = h * d_per_pix;
                            telemetry.addData("the final angle lmao ", angle);
                            telemetry.update();
                            //if (angle < 0) {
                            //    angle = angle - 10;
                            //}
                            Methods.gyroTurn(Variables.TURN_SPEED, angle);
                            sleep(500);
                            Methods.encoderDrive(Variables.DRIVE_SPEED, 350, 350, 100);
                            Methods.gyroTurn(Variables.TURN_SPEED, 55);
                            Methods. encoderDrive(Variables.DRIVE_SPEED, 350, 350, 10);
                            Hardware.markerServo.setPosition(1);
                            sleep(1500);
                            Methods.encoderDrive(Variables.DRIVE_SPEED, -200, -200, 20);
                            tFodDone = true;
                        }
                    }
                    if (tFodDone == false) {
                        Methods. gyroTurnTo(Variables.TURN_SPEED, 135);
                        sleep(500);
                        updatedRecognitions = Variables.tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(Variables.LABEL_GOLD_MINERAL)) {
                                    Hardware. leftDrive.setPower(0);
                                    Hardware.rightDrive.setPower(0);
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addLine("Skipper we did it, we found the gold mineral");
                                    telemetry.addData("Gold getLeft Value: ", recognition.getLeft());
                                    telemetry.addData("Gold getRight Value: ", recognition.getRight());
                                    float getleftval = (int) recognition.getLeft();
                                    float getrightval = (int) recognition.getRight();
                                    float p = (float) (0.5 * (recognition.getLeft() + recognition.getRight()));
                                    telemetry.addData("Gold p Value that was calculated: ", p);
                                    float h = (float) (p - (0.5 * 800));
                                    telemetry.addData("h value calculated ", h);
                                    float angle = h * d_per_pix;
                                    telemetry.addData("the final angle lmao ", angle);
                                    telemetry.update();
                                    //if (angle < 0) {
                                    //    angle = angle - 10;
                                    //}
                                    Methods.gyroTurn(Variables.TURN_SPEED, angle);
                                    sleep(500);
                                    Methods. encoderDrive(Variables.DRIVE_SPEED, 350, 350, 100);
                                    Methods.gyroTurn(Variables.TURN_SPEED, -55);
                                    Methods.encoderDrive(Variables.DRIVE_SPEED, 350, 350, 10);
                                    Hardware.markerServo.setPosition(1);
                                    sleep(1500);
                                    Methods.encoderDrive(Variables.DRIVE_SPEED, -200, -200, 10);
                                    sleep(500);
                                    tFodDone = true;
                                }
                            }
                        }
                    }
                    if (tFodDone == false) {
                        Methods.gyroTurnTo(Variables.TURN_SPEED, 180);
                        Methods.encoderDrive(Variables.DRIVE_SPEED, 650, 650, 5);
                        Hardware.markerServo.setPosition(1);
                        sleep(1500);
                        Methods.encoderDrive(Variables.DRIVE_SPEED, -650, -650, 5);
                    }

                    telemetry.update();
                }
            }
        }
        if (Variables.tfod != null) {
            Variables.tfod.shutdown();
        }
        telemetry.addData("Tfod", "done");
        telemetry.update();


    }
}
