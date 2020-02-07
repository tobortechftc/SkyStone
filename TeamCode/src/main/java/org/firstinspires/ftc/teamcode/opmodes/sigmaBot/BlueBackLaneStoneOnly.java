package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;

@Autonomous(name = "Blue Back Lane", group = "Sigma")
public class BlueBackLaneStoneOnly extends LinearOpMode {
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
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.AUTO_BLUE);
            configuration.apply();
            robot.reset(true);
            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboSigma Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)

        if (robot.intake != null)
            robot.intake.intakeDropInit();
        waitForStart();
        robot.runtime.reset();
        robot.runtimeAuto.reset();

        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
                robot.cameraStoneDetector.getSkystonePositionElementary(telemetry);
                sleep(10000);
//                boolean isBlue = true;
//                sleep(2000);
//                robot.chassis.driveAuto(0.6, 60, +90, 3000);
//                StoneLoc = robot.cameraStoneDetector.getSkystonePositionTF(false);
//                telemetry.addData("StoneLoc",StoneLoc);
//                telemetry.update();
//                sleep(3000);
//                if (StoneLoc == ToboSigma.SkystoneLocation.LEFT) {
//                    robot.chassis.driveAuto(0.6, 60, +90, 3000);
//                } else if (StoneLoc == ToboSigma.SkystoneLocation.RIGHT) {
//                    robot.chassis.driveAuto(0.6, 60, +90, 3000);
//                } else {
//                    robot.chassis.driveAuto(0.6, 60, +90, 3000);
//                }
//                robot.getOneStone();
//                sleep(2000);
//                robot.chassis.rotateTo(0.6, -90);
//
//                robot.chassis.driveAuto(0.6, 120, 0, 4000);

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
