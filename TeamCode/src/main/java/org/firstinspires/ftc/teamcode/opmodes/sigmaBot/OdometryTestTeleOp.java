package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;
import org.firstinspires.ftc.teamcode.components.odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@TeleOp(name="Odometry Test-TeleOp", group="Sigma")
public class OdometryTestTeleOp extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    private EventManager eventManager1;
    private EventManager eventManager2;

    DcMotorEx verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_CM = 307.699557 / 2.54;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboSigma TeleOp runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboSigma robot = new ToboSigma();
        robot.configureLogging("ToboSigma",LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboSigma TeleOp finished configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.NOT_AUTO);
            configuration.apply();
            robot.reset(false, false);
            robot.intake.intakeDropDown();

            eventManager1 = new EventManager(gamepad1, true);
            eventManager2 = new EventManager(gamepad2, true);

            robot.mainTeleOp(eventManager1, eventManager2); // define events for the chassis drive

            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboSigma TeleOp finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.core.set_yield_handler(this); // uses this class as yield handler

        robot.foundationHook.hookUp();
        robot.stoneGrabber.grabberOpen();
        robot.stoneGrabber.outGateClose();

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_CM * 2.54,  75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try {

                eventManager1.processEvents();
                eventManager2.processEvents();
                TaskManager.processTasks();

                telemetry.addData("X Coordinate", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_CM);
                telemetry.addData("Y Coordinate", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_CM);
                telemetry.addData("angle", globalPositionUpdate.returnOrientation() / COUNTS_PER_CM);

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
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }

    public void on_yield() {
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
