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
@Autonomous(name = "Red 2SSIntake", group = "Sigma")
public class RedTwoSSIntake extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;

    protected static int LOG_LEVEL = Log.WARN; // change to Log.INFO to show more messages in log

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;
    private boolean isBlue;

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
                StoneLoc = robot.cameraStoneDetector.getSkystonePositionTF(true, false);
                ToboSigma.SkystoneLocation StoneLoc2 = StoneLoc = robot.cameraStoneDetector.getSkystonePositionElementary(telemetry, false, ToboSigma.AutoTeamColor.AUTO_RED);
                if (StoneLoc2 != ToboSigma.SkystoneLocation.UNKNOWN) {
                    StoneLoc = StoneLoc2;
                }

                if (!opModeIsActive()) return;
                robot.wheelIntakeFirstStone(StoneLoc,false);
                if (!opModeIsActive()) return;
                robot.rotateFoundation(false);
                if (!opModeIsActive()) return;
                robot.wheelIntakeSecondStone(2, StoneLoc == ToboSigma.SkystoneLocation.LEFT ? 3 : (StoneLoc == ToboSigma.SkystoneLocation.CENTER ? 2 : 1), false);
                if (!opModeIsActive()) return;
//                robot.stoneGrabber.lifterDownCombo();
//                if (!opModeIsActive()) return;
                robot.deliverAndPark2SS(false,StoneLoc == ToboSigma.SkystoneLocation.LEFT ? 3 : (StoneLoc == ToboSigma.SkystoneLocation.CENTER ? 2 : 1));
//                double disToRight = robot.chassis.getDistance(SwerveChassis.Direction.RIGHT_HI);
//                if (disToRight > 70 || disToRight < 50)
//                    robot.chassis.driveAuto(0.4, disToRight - 65, +90, 1000);
//                if (!opModeIsActive()) return;
//                robot.chassis.driveAuto(0.8, -120, 0, 3000);
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
