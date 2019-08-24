package org.firstinspires.ftc.teamcode.hardware.Sigma;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;
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
public class CameraMineralDetector extends Logger<CameraMineralDetector> implements Configurable {
    static Logger<CameraMineralDetector> logger = new Logger<>();

    static {
        logger.configureLogging("CameraMineralDetector", Log.VERBOSE);
    }

    private static final String VUFORIA_KEY = "AaaZDWL/////AAAAGYIaD+Gn/UUDhEiR/gcOJxdEJlKEpCOKSLPfhYJfYthUNZ0vnEGm0VGPutkNgRq8bq1ufm3eAySnLhkJQ7d4w6VDT7os5FGPEOGPfsIWMYNAFMdX+wlJo2JCyljeSxQtXUd/YileyfYKBXOl2uFA4KnStCC9WkYTUBrAof3H7RGKorzYixDeOpbmCsf25rayjtAUQrKCwG4j6P5rRdxy7SC//v4VC6NirNwgJ/xn1r02/jbx8vUDrDODGyut9iLk06IzMnrq/P01yKOp48clTw0WIKNmVT7WUQweXy+E1w6xwFplTlPkjC+gzerDOpxHPqYg8RusWD2Y/IMlmnk1yzJba1B9Xf9Ih6BJbm/fVwL4";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void setAdjustmentMode(boolean on) {
        // CameraMineralDetector doesn't need an adjustment mode
        // Method is only declared for completeness of subsystem
    }

    @Override
    public String getUniqueName() {
        return "CameraMineralDetector";
    }

    public void configure(Configuration configuration, boolean useExtCam) {
        logger.verbose("Start Configuration");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (useExtCam) {
            parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "Webcam 1");
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        int tfodMonitorViewId = configuration.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", configuration.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TfodRoverRuckus.TFOD_MODEL_ASSET, TfodRoverRuckus.LABEL_GOLD_MINERAL, TfodRoverRuckus.LABEL_SILVER_MINERAL);

        logger.verbose("CameraMineralDetector status: %s", tfod);

        tfodParameters.minimumConfidence = 0.2;

        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
//        com.vuforia.CameraDevice.getInstance().setField("iso", "800");

        // register CameraMineralDetector as a configurable component
        configuration.register(this);
    }

    public ToboSigma.MineralDetection.SampleLocation getGoldPositionTF(boolean isHanging) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        logger.verbose("Start getGoldPositionTF()");
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            logger.verbose("Start tfod Activation");
            tfod.activate();
            logger.verbose("tfod activate: ", tfod);
        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        int minDistanceFromTop = isHanging ? 320 : 150;

        ToboSigma.MineralDetection.SampleLocation sampleLocation = ToboSigma.MineralDetection.SampleLocation.UNKNOWN;
        int goldXCoord = -1;
        int silverXCoord = -1;
        while (elapsedTime.seconds() < 2 && goldXCoord == -1 && silverXCoord == -1) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() >= 2) {
                        logger.verbose("Starting recognitions");
                        logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
                        int validRecognitions = 0;
                        for (Recognition recognition :
                                updatedRecognitions) {
                            if (recognition.getTop() > minDistanceFromTop) {
                                validRecognitions++;
                            }
                        }
                        logger.verbose("Valid recognitions: %d", validRecognitions);
                        if (validRecognitions >= 2) {
                            for (Recognition recognition :
                                    updatedRecognitions) {
                                if (recognition.getLabel().equals(TfodRoverRuckus.LABEL_GOLD_MINERAL) && recognition.getTop() > minDistanceFromTop) {
                                    goldXCoord = (int) recognition.getLeft();
                                    logger.verbose("Gold X = %d", (int) goldXCoord);
                                    logger.verbose("Gold -Y = %d", (int) recognition.getTop());
                                } else if (recognition.getLabel().equals(TfodRoverRuckus.LABEL_SILVER_MINERAL) && recognition.getTop() > minDistanceFromTop) {
                                    silverXCoord = (int) recognition.getLeft();
                                    logger.verbose("Silver X = %d", (int) silverXCoord);
                                    logger.verbose("Silver -Y = %d", (int) recognition.getTop());
                                }
                                if (goldXCoord != -1 && goldXCoord < silverXCoord) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.LEFT;
                                } else if (goldXCoord != -1 && goldXCoord > silverXCoord) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.CENTER;
                                } else if (goldXCoord == -1 && silverXCoord != -1) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.RIGHT;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }

        switch (sampleLocation) {
            case LEFT:
                logger.verbose("SampleLocation: Left");
            case RIGHT:
                logger.verbose("SampleLocation: Right");
            case CENTER:
                logger.verbose("SampleLocation: Center");
            case UNKNOWN:
                logger.verbose("Sample Location: Unknown");
            default:
                logger.verbose("Sample Location: Unknown");
        }
        return sampleLocation;
    }

    public ToboSigma.MineralDetection.SampleLocation getGoldPositionTF_Titan(boolean isHanging) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        logger.verbose("Start getGoldPositionTF()");
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            logger.verbose("Start tfod Activation");
            tfod.activate();
            logger.verbose("tfod activate: ", tfod);
        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        int minDistanceFromTop;
        if (isHanging)
            minDistanceFromTop = 300;
        else
            minDistanceFromTop = 150;

        ToboSigma.MineralDetection.SampleLocation sampleLocation = ToboSigma.MineralDetection.SampleLocation.UNKNOWN;
        int goldXCoord = -1;
        int silverXCoord = -1;
        while (elapsedTime.seconds() < 2 && goldXCoord == -1 && silverXCoord == -1) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() >= 2) {
                        logger.verbose("Starting recognitions");
                        logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
                        int validRecognitions = 0;
                        for (Recognition recognition :
                                updatedRecognitions) {
                            if (recognition.getTop() > minDistanceFromTop) {
                                validRecognitions++;
                            }
                        }
                        logger.verbose("Valid recognitions: %d", validRecognitions);
                        if (validRecognitions >= 2) {
                            for (Recognition recognition :
                                    updatedRecognitions) {
                                if (recognition.getLabel().equals(TfodRoverRuckus.LABEL_GOLD_MINERAL) && recognition.getTop() > minDistanceFromTop) {
                                    goldXCoord = (int) recognition.getLeft();
                                    logger.verbose("Gold X = %d", (int) goldXCoord);
                                    logger.verbose("Gold -Y = %d", (int) recognition.getTop());
                                } else if (recognition.getLabel().equals(TfodRoverRuckus.LABEL_SILVER_MINERAL) && recognition.getTop() > minDistanceFromTop) {
                                    silverXCoord = (int) recognition.getLeft();
                                    logger.verbose("Silver X = %d", (int) silverXCoord);
                                    logger.verbose("Silver -Y = %d", (int) recognition.getTop());
                                }
                                if (goldXCoord != -1 && goldXCoord < silverXCoord) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.LEFT;
                                } else if (goldXCoord != -1 && goldXCoord > silverXCoord) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.CENTER;
                                } else if (goldXCoord == -1 && silverXCoord != -1) {
                                    sampleLocation = ToboSigma.MineralDetection.SampleLocation.RIGHT;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }

        switch (sampleLocation) {
            case LEFT:
                logger.verbose("SampleLocation: Left");
            case RIGHT:
                logger.verbose("SampleLocation: Right");
            case CENTER:
                logger.verbose("SampleLocation: Center");
            case UNKNOWN:
                logger.verbose("Sample Location: Unknown");
            default:
                logger.verbose("Sample Location: Unknown");
        }
        return sampleLocation;
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
//            line.addData("CameraMineralDetector", "Recog. Count= %d", new Func<Integer>() {
//                @Override
//                public Integer value() {
//
//                }
//            });
//    }
