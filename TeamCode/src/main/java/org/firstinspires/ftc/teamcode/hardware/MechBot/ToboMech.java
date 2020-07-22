package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.MechChassis;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

public class ToboMech extends Logger<ToboMech> implements Robot2 {
    private Telemetry telemetry;
    public MechChassis chassis;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ToboSigma.AutoTeamColor autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

        this.core = new CoreSystem();
        info("RoboRuck configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new MechChassis(core).configureLogging("Mecanum", logLevel); // Log.DEBUG
        chassis.configure(configuration, (autoColor!= ToboSigma.AutoTeamColor.NOT_AUTO));
        info("RoboRuck configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }


    @Override
    public void reset(boolean auto) {
        chassis.reset();
        if (auto) {
            chassis.setupTelemetry(telemetry);
        }
    }


    @MenuEntry(label = "TeleOp", group = "Test Chassis")
    public void mainTeleOp(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() { // Left-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY))>0.2 )
                    return; // avoid conflicting drives

                // Left joystick for forward/backward and turn
                if (Math.abs(currentY)>0.2) { // car mode
                    chassis.carDrive(currentY*Math.abs(currentY), currentX);
                }else if (Math.abs(currentX) > 0.2) {
                    chassis.turn((currentX > 0 ? 1 : -1), Math.abs(currentX * currentX) * chassis.powerScale());
                } else if (Math.abs(currentY)>0.2) {
                    chassis.yMove((currentY>0?1:-1), Math.abs(currentY * currentY) * chassis.powerScale());
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);

        em.onStick(new Events.Listener() { // Right-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY))>0.2 )
                    return; // avoid conflicting drives
                double left_x = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY);
                // right joystick for free crabbing
                if (Math.abs(left_x)>0.1 && Math.abs(currentY)>0.1) {
                    // car drive
                    chassis.carDrive(currentY*Math.abs(currentY), left_x);
                } else if (Math.abs(currentX)+Math.abs(currentY)>0.2) {
                    double lsx = currentX;
                    double lsy = currentY;
                    double power_lf = lsy+lsx;
                    double power_lb = lsy-lsx;
                    double power_rf = lsy-lsx;
                    double power_rb = lsy+lsx;

                    power_lf = Range.clip(power_lf, -1, 1);
                    power_lb = Range.clip(power_lb, -1, 1);
                    power_rf = Range.clip(power_rf, -1, 1);
                    power_rb = Range.clip(power_rb, -1, 1);

                    power_lf *= Math.abs(power_lf)* chassis.powerScale();
                    power_lb *= Math.abs(power_lb)* chassis.powerScale();
                    power_rf *= Math.abs(power_rf)* chassis.powerScale();
                    power_rb *= Math.abs(power_rb)* chassis.powerScale();
                    chassis.freeStyle(power_lf, power_rf, power_lb, power_rb);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        if (chassis==null) return;
        /*
        line.addData(" | Odometry (vl,vr,h) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%5d,%5d,%5d)\n", chassis.verticalLeftEncoder().getCurrentPosition(),
                        chassis.verticalRightEncoder().getCurrentPosition(),chassis.horizontalEncoder().getCurrentPosition());
            }
        });
        line.addData("Odo (x, y, angle) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%4.0f, %4.0f, %4.0f)\n", chassis.odo_x_pos(),chassis.odo_y_pos(),chassis.odo_heading());
            }
        });

        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });
        */
    }

    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }
}
