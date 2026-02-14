package org.firstinspires.ftc.teamcode.Atlas.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name="Launcher Logger", group="Test")
public class AtlasLauncherLogger extends OpMode {

    private AtlasFunctions fn;
    private ElapsedTime t = new ElapsedTime();

    private static final int MAX_SAMPLES = 9000; // ~30s at 100Hz-ish
    private int n = 0;

    private double[] timeS = new double[MAX_SAMPLES];
    private double[] targetRpm = new double[MAX_SAMPLES];
    private double[] measRpm = new double[MAX_SAMPLES];
    private int[] marker = new int[MAX_SAMPLES];
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastA = false;
    private boolean lastX = false;

    private int markerNow = 0;
    private boolean running = false;

    @Override
    public void init() {
        AtlasFunctions.ShooterConfig cfg = new AtlasFunctions.ShooterConfig(
                28,     // ticksPerRev
                3450 ,   // holdRpm (use your real hold)
                2500,   // close
                3500    // far
        );
        fn = new AtlasFunctions(hardwareMap, cfg);

        telemetry.addLine("X: start/stop logging");
        telemetry.addLine("A: marker pulse (press when ring is fed)");
        telemetry.update();
    }

    @Override
    public void start() {
        t.reset();
        fn.shooterOff();
    }

    @Override
    public void loop() {
        boolean intakeHeld = gamepad1.left_trigger > 0.5;
        boolean yNow = gamepad1.y;
        boolean bNow = gamepad1.b;

        if (yNow && !lastY) fn.setPreset(AtlasFunctions.RangePreset.CLOSE);
        if (bNow && !lastB) fn.setPreset(AtlasFunctions.RangePreset.FAR);
        lastY = yNow;
        lastB = bNow;

        if (intakeHeld) {
            fn.intakeOn();
            fn.indexerFeed();

        } else {
            fn.intakeOff();
            fn.indexerOff();
        }

        boolean xNow = gamepad1.x;
        if (xNow && !lastX) {
            running = !running;
            if (running) {
                n = 0;
                t.reset();
                markerNow = 0;
                fn.shooterHold(); // step to hold when logging starts
            } else {
                fn.shooterOff();
                writeCsv();
            }
        }
        lastX = xNow;

        boolean aNow = gamepad1.a;
        if (aNow && !lastA) {
            markerNow = 1; // one-sample marker pulse
        }
        lastA = aNow;

        // Sample
        if (running && n < MAX_SAMPLES) {
            timeS[n] = t.seconds();
            targetRpm[n] = fn.getTargetRpm();
            measRpm[n] = fn.getMeasuredRpm();
            marker[n] = markerNow;
            n++;
            markerNow = 0;
        }

        telemetry.addData("Logging", running);
        telemetry.addData("Samples", n);
        telemetry.addData("TargetRPM", "%.1f", fn.getTargetRpm());
        telemetry.addData("MeasRPM", "%.1f", fn.getMeasuredRpm());
        telemetry.addLine("X toggles logging. A marks feed moment.");
        telemetry.update();
    }

    @Override
    public void stop() {
        fn.shooterOff();
        if (n > 0) writeCsv();
    }

    private void writeCsv() {
        File dir = new File("/sdcard/FIRST/");
        if (!dir.exists()) dir.mkdirs();

        String name = String.format(Locale.US, "launcher_log_%d.csv", System.currentTimeMillis());
        File f = new File(dir, name);

        try (FileWriter w = new FileWriter(f)) {
            w.write("t_s,target_rpm,meas_rpm,marker\n");
            for (int i = 0; i < n; i++) {
                w.write(String.format(Locale.US, "%.4f,%.2f,%.2f,%d\n",
                        timeS[i], targetRpm[i], measRpm[i], marker[i]));
            }
            telemetry.addData("Saved", f.getAbsolutePath());
        } catch (IOException e) {
            telemetry.addData("CSV write failed", e.getMessage());
        }
    }
}