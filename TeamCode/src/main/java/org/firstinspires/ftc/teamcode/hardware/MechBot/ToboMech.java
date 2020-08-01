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

import static java.lang.Thread.sleep;

public class ToboMech extends Logger<ToboMech> implements Robot2 {
    private Telemetry telemetry;
    public MechChassis chassis;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation


    public double auto_chassis_power = .7;
    public double auto_chassis_dist = 150;
    public double auto_chassis_heading = -90;
    public double auto_chassis_power_slow = .4;
    public double auto_chassis_align_power = .22;

    public double auto_rotate_degree = 90;



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
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY))> 0.2 )
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


    //@MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double power = Math.max(Math.abs(currentX), Math.abs(currentY));
                double heading = toDegrees(currentX, currentY);
                debug("testStraight(): x: %+.2f, y: %+.2f, pow: %+.3f, head: %+.1f",
                        currentX, currentY, power, heading);

                if (source.isPressed(Button.LEFT_BUMPER) || source.isPressed(Button.RIGHT_BUMPER)) {
                    // constrain to 45 degree diagonals
                    heading = Math.signum(heading) * (Math.abs(heading) < 90 ? 45 : 135);
                } else {
                    // constrain to 90 degrees
                    heading = Math.round(heading / 90) * 90;
                }

                // adjust heading / power for driving backwards
                if (heading > 90) {
                    chassis.driveStraight( power, -30, heading - 180, 3);
                } else if (heading < -90) {
                    chassis.driveStraight( power, -30, heading + 180, 3);
                } else {
                    chassis.driveStraight(power, 30,  heading, 3);
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);
    }

    @MenuEntry(label = "Auto Rotation", group = "Test Chassis")
    public void testRotationSkyStone(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- degree(%.2f)", auto_rotate_degree).setRetained(true);
        chassis.setupTelemetry(telemetry);
        chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.rotateToOld(auto_chassis_power, auto_rotate_degree, 5000);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                    if (auto_rotate_degree > 150) auto_rotate_degree = 150;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 10;
                    if (auto_rotate_degree < -150) auto_rotate_degree = -150;
                }
            }
        }, new Button[]{Button.B});

    }

     @MenuEntry(label = "New Auto Straight", group = "Test Chassis")
    public void testStraightNewSkyStone(EventManager em) {
        if (Thread.interrupted()) return;

        try {

            chassis.driveStraight(.6, 50, -90, 10000);
            sleep(500);
            chassis.driveStraight(.6, 50, 90, 10000);
            sleep(500);
            chassis.driveStraight(.6, 50, 0, 10000);
            sleep(500);
            chassis.driveStraight(.6, -50, 0, 10000);
        } catch (InterruptedException e) {

        }

    }


}
