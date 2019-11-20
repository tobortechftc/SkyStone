package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;
@Disabled
@Autonomous(name = "Sigma-foundation testing", group = "Sigma")
public class FoundationHookTesting extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .3;

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
        List<Recognition> updatedRecognitions = null;
        if (robot.cameraStoneDetector.getTfod() != null) {
            updatedRecognitions = robot.cameraStoneDetector.getTfod().getUpdatedRecognitions();
        }
        int robot_pos = 1;

        waitForStart();
        robot.runtime.reset();
        // run until the end of the match (driver presses STOP or timeout)

        while(opModeIsActive()){
            telemetry.addData("mag touch", robot.foundationHook.touchingState());
            telemetry.addData("range sensor", robot.foundationHook.rangeReading());
            telemetry.update();
        }
        /**
        //start "foundation repositioning routine"
        robot.chassis.driveStraight(-0.3, 0);
        while (robot.foundationHook.touchingState() != 0b11) {
            //waiting
            telemetry.addData("touch right", robot.foundationHook.rightTouch.isPressed());
            telemetry.addData("touch left", robot.foundationHook.leftTouch.isPressed());
            telemetry.update();
        }
        robot.chassis.stop();
        robot.foundationHook.hookDown();
        robot.chassis.driveStraightAuto(0.3, 20, 0, 4000);
        //end "foundation repositioning routine"
         **/
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
