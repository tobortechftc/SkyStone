package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@Autonomous(name = "Red 2SS", group = "Sigma")
public class RedTwoSS extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboSigma Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboSigma robot = new ToboSigma();
        robot.configureLogging("ToboSigma", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboSigma Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.AUTO_RED);
            configuration.apply();
            robot.reset(true);
            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboSigma Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());
        if (robot.intake != null)
            robot.intake.intakeDropInit();
        waitForStart();
        robot.runtime.reset();
        robot.runtimeAuto.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
                boolean isBlue = false;
                StoneLoc = robot.cameraStoneDetector.getSkystonePositionTF(true);
                ToboSigma.SkystoneLocation StoneLoc2 = robot.cameraStoneDetector.getSkystonePositionElementary(telemetry, false, ToboSigma.AutoTeamColor.AUTO_RED);
                if (StoneLoc2 != ToboSigma.SkystoneLocation.UNKNOWN) {
                    StoneLoc = StoneLoc2;
                }
                int ss_pos = robot.getFirstSkyStoneDefense(StoneLoc, isBlue, false);
                if (opModeIsActive()) {
                    robot.rotateFoundationNew(isBlue);
                }
                int count = 2;
                if ((robot.runtimeAuto.seconds() < 19.5) && opModeIsActive()) {//may be too large - TYPICALLY AROUND 17-18
                    if (ss_pos == 3) {
                        if (opModeIsActive()) {
                            robot.getWallStone(isBlue);
                        }
                        if (robot.runtimeAuto.seconds() < 29.0) {
                            robot.deliverAndPark2SS(false,ss_pos);//?????????
                        }
                    } else {
                        if (opModeIsActive()) {
                            robot.getAnotherSkyStoneNew(ss_pos, count, isBlue, true);
                        }
                        if ((robot.runtimeAuto.seconds() < 29.0)&&opModeIsActive()) {
                            robot.park2SS(true);
                        }
                    }
                } else {
                    if (opModeIsActive()) {
                        robot.parkAfterRotateNew(isBlue);
                    }
                }

            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
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
