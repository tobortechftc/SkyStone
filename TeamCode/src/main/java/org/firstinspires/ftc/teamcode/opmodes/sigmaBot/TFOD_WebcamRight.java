/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
@TeleOp(name = "TFOD:WebcamRight", group = "Test")
public class TFOD_WebcamRight extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    //    private static final String VUFORIA_KEY = "AaaZDWL/////AAAAGYIaD+Gn/UUDhEiR/gcOJxdEJlKEpCOKSLPfhYJfYthUNZ0vnEGm0VGPutkNgRq8bq1ufm3eAySnLhkJQ7d4w6VDT7os5FGPEOGPfsIWMYNAFMdX+wlJo2JCyljeSxQtXUd/YileyfYKBXOl2uFA4KnStCC9WkYTUBrAof3H7RGKorzYixDeOpbmCsf25rayjtAUQrKCwG4j6P5rRdxy7SC//v4VC6NirNwgJ/xn1r02/jbx8vUDrDODGyut9iLk06IzMnrq/P01yKOp48clTw0WIKNmVT7WUQweXy+E1w6xwFplTlPkjC+gzerDOpxHPqYg8RusWD2Y/IMlmnk1yzJba1B9Xf9Ih6BJbm/fVwL4";
    private static final String VUFORIA_KEY = "AS0FKrL/////AAABmTcBCNs1gE8uh4tntGA7HSgXRT5npDQpV2pw5tlqbZCI6WJQRf0bKf458A218bGkQJCWkJzvJy6UtNnhziraRVDDZSnTSZGSD7s3WN9jNYqBiSoO3CVE6FU2dX1yuJNa1zfiEhcGT8ChTd+kucE/q3sXsy/nw1KqlW/7uEaEeRwaCPseqsbNrc1HZ1bi18PzwQpCaypDruqqVEyZ3dvTqDmjPg7WFBe2kStPR/qTtcLSXdE804RxxkgTGUtDMIG7TTbAdirInGVZw2p2APZKAQdYofYW2E0Ss5hZCeL55zflSuQK0QcW1sAyvaTUMd/fDse4FgqxhnfK0ip0Kc+ZqZ6XJpof/Nowwxv3IgDWZJzO";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);


    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        /*
        int total_stones = 0;
        int big_ss = 0;
        int good_ss = 0;
        int ns = 0;
        int ss_left = 0;
        int ss_cener = 0;
        int ss_right = 0;
        double stoneYpos = 0;
         */
        while (opModeIsActive()) {
            if (tfod == null) {
                continue;
            }

            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.startTime();
            while (elapsedTime.seconds() < 0.3) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                int i=0;
                if (updatedRecognitions == null || updatedRecognitions.size() < 1) {
                    continue;
                }
                for (Recognition recognition :
                        updatedRecognitions) {
                    i++;
                    double width = recognition.getWidth();
                    double height = recognition.getHeight();
                    double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    telemetry.addData("Stone","%d:: wid=%.1f,he=%.1f,ang=%.1f",i,width,height,angle);
                }
                telemetry.update();
            }
            /*
            boolean redSide = false;
            int max_stone_width = 250;
            int min_stone_width = 150;
            int left_center_border_x = 200;
            int center_right_border_x = 400;
            ToboSigma.SkystoneLocation skystoneLocation= ToboSigma.SkystoneLocation.UNKNOWN;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null) {
                continue;
            }
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                double pos = (recognition.getRight() + recognition.getLeft()) / 2;
                double skystone_width = recognition.getRight() - recognition.getLeft();
                stoneYpos = (recognition.getTop() + recognition.getBottom()) / 2;
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("pos/width/Ypos (%d)", i), "%.1f /%.1f/%.1f",
                                pos, skystone_width, stoneYpos);
                telemetry.addData(String.format("top/bottom/left/right (%d)", i), "%.1f /%.1f/%.1f/%.1f",
                        recognition.getTop(), recognition.getBottom(), recognition.getLeft(), recognition.getRight());
            }
            telemetry.update();

            if (updatedRecognitions.size() < 1) {
                continue;
            }
            //logger.verbose("Starting recognitions");
            //logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
            int validRecognitions = 0;
            if (ns>1000)
                break;
            for (Recognition recognition :
                    updatedRecognitions) {
                if (recognition.getLabel() == "Stone") {
                    ns++;
                    continue;
                }
                double width = recognition.getRight() - recognition.getLeft();
                if (width < max_stone_width && width > min_stone_width) {
                    validRecognitions++;
                }
            }
            //logger.verbose("Valid recognitions: %d", validRecognitions);
            //if (validRecognitions!=1) { // not detect just one skystone
            //    continue;
            //}
            for (Recognition recognition :
                    updatedRecognitions) {
                total_stones++;
                if (recognition.getLabel() == "Stone") {
                    continue;
                }
                double pos = (recognition.getRight() + recognition.getLeft()) / 2;
                double skystone_width = recognition.getRight() - recognition.getLeft();
                if (skystone_width>320) { // stone detected is twice as regular, assume the skystone is on the right half
                    // pos = (pos+recognition.getRight())/2;
                    big_ss++;
                } else {
                    good_ss++;
                }
                if (redSide) {
                    if (pos < left_center_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                        ss_right++;
                    } else if (pos > center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                        ss_cener++;
                    } else if (pos >= left_center_border_x && pos <= center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                        ss_left++;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                } else {
                    if (pos < left_center_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                        ss_cener++;
                    } else if (pos > center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                        ss_left++;
                    } else if (pos >= left_center_border_x && pos <= center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;//big stone
                        ss_right++;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                }
                telemetry.addData("Skystone Position: ", skystoneLocation);
                telemetry.update();

            }

            telemetry.update();
            if (big_ss+good_ss>=100) {
                break;
            }
            */
        }
        /*
        telemetry.addLine().addData("Result", "L=%d,C=%d,R=%d,good=%d,big=%d,total=%d",
                ss_left,ss_cener,ss_right,good_ss,big_ss,total_stones);
        telemetry.update();
         */
        sleep(10000);
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "WebcamRight");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
