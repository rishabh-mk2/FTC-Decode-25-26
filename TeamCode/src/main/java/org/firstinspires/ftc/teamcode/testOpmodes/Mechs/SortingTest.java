package org.firstinspires.ftc.teamcode.testOpmodes.Mechs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Config
@TeleOp(name = "Sorting Test", group = "TeleOp")
public class SortingTest extends OpMode {

    // Sensors
    RevColorSensorV3 frontSensor1, frontSensor2;
    RevColorSensorV3 backRightSensor1, backRightSensor2;
    RevColorSensorV3 backLeftSensor1, backLeftSensor2;

    // Distance thresholds (mm)
    public static double FRONT_THRESHOLD = 45;
    public static double BACK_THRESHOLD = 15;

    // Green detection threshold
    // TODO: EDIT THIS
    public static double GREEN_THRESHOLD = 200;

    // Target sort string
    private String targetSort = "OOO";

    // For detecting button release
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastUp = false;

    @Override
    public void init() {

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1 = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2 = hardwareMap.get(RevColorSensorV3.class, "bl2");

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // detect current sort

        char front = detectPosition(frontSensor1, frontSensor2, FRONT_THRESHOLD);
        char backRight = detectPosition(backRightSensor1, backRightSensor2, BACK_THRESHOLD);
        char backLeft = detectPosition(backLeftSensor1, backLeftSensor2, BACK_THRESHOLD);

        String currentSort = "" + front + backRight + backLeft;


        // set targetSort

        if (gamepad1.dpadDownWasReleased()) {
            targetSort = "GPP";
        }
        if (gamepad1.dpadLeftWasReleased()) {
            targetSort = "PGP";
        }
        if (gamepad1.dpadUpWasReleased()) {
            targetSort = "PPG";
        }


        // generate the 3 cyclic permutations

        String perm0 = currentSort;
        String perm1 = currentSort.substring(1) + currentSort.charAt(0);
        String perm2 = currentSort.substring(2) + currentSort.substring(0, 2);


        // score each permutation

        int score0 = scoreMatch(perm0, targetSort);
        int score1 = scoreMatch(perm1, targetSort);
        int score2 = scoreMatch(perm2, targetSort);


        // pick best permutation

        int bestRotation = 0;
        int bestScore = score0;

        if (score1 > bestScore) {
            bestScore = score1;
            bestRotation = 1;
        }

        if (score2 > bestScore) {
            bestScore = score2;
            bestRotation = 2;
        }

        // bestRotation which lead to 3 cases in which for each i rotate the spindexer by some amount to sort

        // telemetry updates

        telemetry.addData("Current Sort", currentSort);
        telemetry.addData("Target Sort", targetSort);
        telemetry.addData("Perm0", perm0 + " Score: " + score0);
        telemetry.addData("Perm1", perm1 + " Score: " + score1);
        telemetry.addData("Perm2", perm2 + " Score: " + score2);
        telemetry.addData("Best Rotation (0,1,2)", bestRotation);
        telemetry.update();
    }


    // =========================
    // SENSOR DETECTION LOGIC
    // =========================

    private char detectPosition(RevColorSensorV3 s1, RevColorSensorV3 s2, double threshold) {

        double d1 = s1.getDistance(DistanceUnit.MM);
        double d2 = s2.getDistance(DistanceUnit.MM);

        boolean valid1 = d1 <= threshold;
        boolean valid2 = d2 <= threshold;

        // REQUIRE BOTH sensors to satisfy threshold
        if (!(valid1 && valid2)) {
            return 'O';
        }

        // Choose the closer one
        RevColorSensorV3 chosen = (d1 <= d2) ? s1 : s2;

        double greenValue = chosen.green();

        if (greenValue >= GREEN_THRESHOLD) {
            return 'G';
        } else {
            return 'P';
        }
    }


    // =========================
    // STRING SCORING
    // =========================

    private int scoreMatch(String a, String b) {
        int score = 0;
        for (int i = 0; i < 3; i++) {
            if (a.charAt(i) == b.charAt(i)) {
                score++;
            }
        }
        return score;
    }


    @Override
    public void stop() {
    }
}
