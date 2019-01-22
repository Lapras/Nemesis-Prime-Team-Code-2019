/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;
@Disabled
@Autonomous(name = "Tensorflow", group = "Concept")

public class tensorBoi extends LinearOpMode {
    public Variables Variables = new Variables();
    public Methods Methods = new Methods();
    public Hardware Hardware = new Hardware();
    @Override
    public void runOpMode() {
        Hardware.hardwareMap();
        Methods.initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            Methods. initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        waitForStart();
        if (opModeIsActive()) {
            if (Variables.tfod != null) {
                Variables.tfod.activate();
            }
            Methods.gyroTurn(Variables.BIG_TURN, -20);
            while (opModeIsActive()) {
                if (Variables.tfod != null) {
                    List<Recognition> updatedRecognitions = Variables.tfod.getUpdatedRecognitions();
                    Hardware.leftDrive.setPower(0.1);
                    Hardware.rightDrive.setPower(-0.1);
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        final int fov = 78;
                        final float d_per_pix = (float) 0.040625;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(Variables.LABEL_GOLD_MINERAL)) {
                                if (recognition.getLeft() == 0) {
                                    Hardware.leftDrive.setPower(0);
                                    Hardware.rightDrive.setPower(0);
                                    Methods.gyroTurn(Variables.BIG_TURN, 20);
                                } else if (recognition.getRight() == 0) {
                                    Hardware.leftDrive.setPower(0);
                                    Hardware.rightDrive.setPower(0);
                                    Methods.gyroTurn(Variables.SMALL_TURN, -20);
                                } else {
                                    Hardware.leftDrive.setPower(0);
                                    Hardware.rightDrive.setPower(0);
                                }
                                float getleftval = (int) recognition.getLeft();
                                float getrightval = (int) recognition.getRight();
                                float p = (float) (0.5 * (recognition.getLeft() + recognition.getRight()));
                                telemetry.addData("Gold p Value that was calculated: ", p);
                                float h = (float) (p - (0.5 * 800));
                                telemetry.addData("h value calculated ", h);
                                float angle = h * d_per_pix;
                                telemetry.addData("the final angle lmao ", angle);
                                telemetry.update();
                                Methods.gyroTurn(Variables.BIG_TURN, angle);
                                sleep(500);
                                Methods.encoderDrive(Variables.DRIVE_SPEED, 350, 350, 100);
                                Methods.encoderDrive(Variables.DRIVE_SPEED, -350, -350, 100);
                                sleep(500);
                            }
                            break;
                        }
                        telemetry.update();
                    }
                }
            }
        }
        if (Variables.tfod != null) {
            Variables.tfod.shutdown();
        }
    }
}