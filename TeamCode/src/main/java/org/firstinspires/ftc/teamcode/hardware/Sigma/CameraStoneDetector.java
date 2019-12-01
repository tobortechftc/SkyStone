package org.firstinspires.ftc.teamcode.hardware.Sigma;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;
//import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
//import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class CameraStoneDetector extends Logger<CameraStoneDetector> implements Configurable {
    static Logger<CameraStoneDetector> logger = new Logger<>();

    static {
        logger.configureLogging("CameraStoneDetector", Log.VERBOSE);
    }
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AS0FKrL/////AAABmTcBCNs1gE8uh4tntGA7HSgXRT5npDQpV2pw5tlqbZCI6WJQRf0bKf458A218bGkQJCWkJzvJy6UtNnhziraRVDDZSnTSZGSD7s3WN9jNYqBiSoO3CVE6FU2dX1yuJNa1zfiEhcGT8ChTd+kucE/q3sXsy/nw1KqlW/7uEaEeRwaCPseqsbNrc1HZ1bi18PzwQpCaypDruqqVEyZ3dvTqDmjPg7WFBe2kStPR/qTtcLSXdE804RxxkgTGUtDMIG7TTbAdirInGVZw2p2APZKAQdYofYW2E0Ss5hZCeL55zflSuQK0QcW1sAyvaTUMd/fDse4FgqxhnfK0ip0Kc+ZqZ6XJpof/Nowwxv3IgDWZJzO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void setAdjustmentMode(boolean on) {
        // CameraStoneDetector doesn't need an adjustment mode
        // Method is only declared for completeness of subsystem
    }

    @Override
    public String getUniqueName() {
        return "CameraStoneDetector";
    }
    public TFObjectDetector getTfod() { return tfod; }

    public void configure(Configuration configuration, ToboSigma.CameraSource cameraSource) {
        logger.verbose("Start Configuration");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (cameraSource== ToboSigma.CameraSource.WEBCAM_RIGHT) {
            parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "WebcamRight");
        } else if (cameraSource== ToboSigma.CameraSource.WEBCAM_LEFT) {
            parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "WebcamLeft");
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        int tfodMonitorViewId = configuration.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", configuration.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        logger.verbose("CameraStoneDetector status: %s", tfod);

        tfodParameters.minimumConfidence = 0.6;

        if (cameraSource== ToboSigma.CameraSource.INTERNAL)
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
//        com.vuforia.CameraDevice.getInstance().setField("iso", "800");

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            logger.verbose("Start tfod Activation");
            tfod.activate();
            logger.verbose("tfod activate: ", tfod);
        }

        // register CameraStoneDetector as a configurable component
        configuration.register(this);
    }

    public ToboSigma.SkystoneLocation getSkystonePositionTF(boolean redSide) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        int left_center_border_x = 200;
        int center_right_border_x = 400;

        int min_stone_width = 150;
        int max_stone_width = 250;
        int large_stone_width = 320;

        logger.verbose("Start getGoldPositionTF()");

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        ToboSigma.SkystoneLocation skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
        //int goldXCoord = -1;
        //int silverXCoord = -1;
        if(tfod==null){
            return ToboSigma.SkystoneLocation.UNKNOWN;
        }

        while (elapsedTime.seconds() < 2 && skystoneLocation == ToboSigma.SkystoneLocation.UNKNOWN) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions.size() < 1) {
                continue;
            }
            //logger.verbose("Starting recognitions");
            //logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
            int validRecognitions = 0;
            for (Recognition recognition :
                    updatedRecognitions) {
                double width = recognition.getRight() - recognition.getLeft();
                if (width < max_stone_width && width > min_stone_width) {
                    validRecognitions++;
                }
            }
            //logger.verbose("Valid recognitions: %d", validRecognitions);
            if (validRecognitions < 1 || validRecognitions > 3) {
                continue;
            }
            for (Recognition recognition :
                    updatedRecognitions) {
                if (recognition.getLabel() == "Stone") {
                    continue;
                }
                double pos = (recognition.getRight() + recognition.getLeft()) / 2;
                double skystone_width = recognition.getRight() - recognition.getLeft();
                if (skystone_width>large_stone_width) { // stone detected is twice as regular, assume the skystone is on the right half
                    pos = (pos+recognition.getRight())/2;
                }
                if (redSide) {
                    if (pos < left_center_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                    } else if (pos > center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                    } else if (pos >= left_center_border_x && pos <= center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                } else {
                    if (pos < left_center_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                    } else if (pos > center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                    } else if (pos >= left_center_border_x && pos <= center_right_border_x) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }

        switch (skystoneLocation) {
            case LEFT:
                logger.verbose("SampleLocation: Left");
                break;
            case RIGHT:
                logger.verbose("SampleLocation: Right");
                break;
            case CENTER:
                logger.verbose("SampleLocation: Center");
                break;
            case UNKNOWN:
                logger.verbose("Sample Location: Unknown");
                break;
            default:
                logger.verbose("Sample Location: Unknown");
                break;
        }
        return skystoneLocation;
    }

}

/**
 * Set up telemetry lines for chassis metrics
 * Shows current motor power, orientation sensors,
 * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
 * and servo position for each wheel
 */
//    public void setupTelemetry(Telemetry telemetry) {
//        Telemetry.Line line = telemetry.addLine();
//        if ()
//            line.addData("CameraStoneDetector", "Recog. Count= %d", new Func<Integer>() {
//                @Override
//                public Integer value() {
//
//                }
//            });
//    }
