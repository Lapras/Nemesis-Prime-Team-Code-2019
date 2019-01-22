package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;
@Disabled
@Autonomous(name="Auton Tfod", group="Minibot")
public class AutonTFOD extends LinearOpMode {
    public Variables Variables = new Variables();
    public Methods Methods = new Methods();
    public Hardware Hardware = new Hardware();
    public void runOpMode() {
        Methods.initVuforia();
        Hardware.hardwareMap();
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
        Methods.elevatorDrive(1, 8.5, 4);
        Methods.encoderDrive(Variables.DRIVE_SPEED, 100, 100, 4);
        Methods.elevatorDrive(1, -8.5, 4);
        Methods.encoderDrive(Variables.DRIVE_SPEED, -100, -100, 4);
        if (Variables.tfod != null) {
            Variables.tfod.activate();
        }
        Methods.gyroTurn(Variables.TURN_SPEED, -20);
        boolean tFodDone = false;
        while (opModeIsActive() && tFodDone == false) {
            if (Variables.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = Variables.tfod.getUpdatedRecognitions();
                Hardware.leftDrive.setPower(0.075);
                Hardware.rightDrive.setPower(-0.075);
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    final int fov = 78;
                    final float d_per_pix = (float) 0.040625;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(Variables.LABEL_GOLD_MINERAL)) {
                            Hardware.leftDrive.setPower(0);
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
                            sleep(30000);
                            //if (angle < 0) {
                            //    angle = angle - 10;
                            //}

                            Methods.gyroTurn(Variables.TURN_SPEED, angle);
                            sleep(500);
                            Methods.encoderDrive(Variables.DRIVE_SPEED, 350, 350, 100);
                            Methods.encoderDrive(Variables.DRIVE_SPEED, -350, -350, 100);
                            sleep(500);
                        }
                    }
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