/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;
import java.util.Locale;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The four vision target are located on the two walls closest to the audience, facing in.
 * The Gears and the Tools are on the RED side of the field, and the Wheels and the Legos are on the Blue side.
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.  David Has Licence Key
 */

@Autonomous(name = "Concept: Vuforia Navigation", group = "Concept")
@Disabled
public class VuforiaNavigation extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";
    public static final int BLUE_MIDDLE_TARGET_INDEX = 0;
    public static final int RED_FAR_TARGET_INDEX = 1;
    public static final int BLUE_FAR_TARGET_INDEX = 2;
    public static final int RED_MIDDLE_TARGET_INDEX = 3;
    public static final String[] TARGET_NAMES = {"Blue Mid (Wheels)", "Red Far (Tools)", "Blue Far(Legos)", "Red Mid (Gears)"};

    private OpenGLMatrix lastLocation = null;
    private OpenGLMatrix[] locationRelativeToTargets = null;
    private boolean[] targetsVisible = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables visionTargets;
    private boolean vuforiaReady = false;
    private boolean vuforiaActivated = false;
    private boolean isTargetVisible = false;

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    public static String format(OpenGLMatrix transformationMatrix) {
        float[] data = transformationMatrix.getTranslation().getData();
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return String.format(Locale.ENGLISH, "(%+05.1f, %+05.1f, %+05.1f), rot.(z-axis) %+03.1f, %s", data[0], data[1], data[2], orientation.thirdAngle, orientation.toString());
    }

    /**
     * UNTESTED
     *
     * @param transformationMatrix the OpenGLMatrix to extract data from
     * @return an array with the relevant data, in the order [x coordinate, y coordinate, z axis rotation]
     */
    public static double[] extractData(OpenGLMatrix transformationMatrix) {
        double[] data = new double[3];
        float[] translationData = transformationMatrix.getTranslation().getData();

        data[0] = translationData[0];
        data[1] = translationData[1];
        data[2] = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        return data;
    }

    public boolean isVuforiaReady() {
        return vuforiaReady;
    }

    public boolean isVuforiaActivated() {
        return vuforiaActivated;
    }

    public VuforiaTrackables getVisionTargets() {
        return visionTargets;
    }

    // y axis is the heading
    //930 is the ~2ft
    public OpenGLMatrix getLastLocation() {
        return lastLocation;
    }

    public OpenGLMatrix[] getLocationRelativeToTargets() {
        return locationRelativeToTargets;
    }

    public boolean[] getTargetsVisible() {
        return targetsVisible;
    }

    /**
     * Start tracking Vuforia images
     */
    public boolean activateTracking() {
        // Start tracking any of the defined targets
        if (vuforiaReady && !vuforiaActivated) {
            visionTargets.activate();
            vuforiaActivated = true;
        }
        return vuforiaReady;
    }

    public void deactivateTracking() {
        if (vuforiaReady && vuforiaActivated) {
            visionTargets.deactivate();

            //invalidate any remaining data
            lastLocation = null;
            for (int i = 0; i < visionTargets.size(); i++) {
                locationRelativeToTargets[i] = null;
                targetsVisible[i] = false;
            }
            isTargetVisible = false;

            vuforiaActivated = false;
        }
    }

    public void initializeVuforia() {
        vuforiaReady = false;
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = RobotHardware.VUFORIA_LICENSE_KEY; //David has key
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = true;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager.
         */
        visionTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable blueMidTarget = visionTargets.get(BLUE_MIDDLE_TARGET_INDEX);
        blueMidTarget.setName("Blue Middle Target");  // Wheels

        VuforiaTrackable redFarTarget = visionTargets.get(RED_FAR_TARGET_INDEX);
        redFarTarget.setName("Red Far Target");  // Tools

        VuforiaTrackable blueFarTarget = visionTargets.get(BLUE_FAR_TARGET_INDEX);
        blueFarTarget.setName("Blue Far Target"); // Legos

        VuforiaTrackable redMidTarget = visionTargets.get(RED_MIDDLE_TARGET_INDEX);
        redMidTarget.setName("Red Middle Target"); // Gears


        /** We don't need to do this, {@Link VuforiaTrackables} already extends List, we just iterate over it*/
//        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(visionTargets);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="Wheels" size="254.000000 184.154922" />
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch = 25.4f;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmPhoneXSlide = 5.4f * mmPerInch;
        float mmPhoneYSlide = 3.3f * mmPerInch; //check with nina/thomas
        float mmPhoneZSlide = 6.3f * mmPerInch; //check with nina/thomas

        float mmTargetHeight = (1.5f + 8.5f / 2f) * mmPerInch; // from the andymark setup guide, 1.5 inches up, 8.5 inch paper
        float mmMidTargetSlide = 12f * mmPerInch; // 1/2 a 2ft tile over
        float mmFarTargetSlide = 36 * mmPerInch;  // 3/2 a 2ft tile over
        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         */

        /*
         * To place the Gears Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it 90 around the field's Z axis to face it away from the audience.
         * - Next we translate it back along the X axis towards the red audience wall
         * - Finally, we translate it up along the Z axis, and back along the Y axis.
         */
        OpenGLMatrix redMiddleTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, -mmMidTargetSlide, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redMidTarget.setLocation(redMiddleTargetLocationOnField);
//        RobotLog.ii(TAG, "Red Target=%s", format(redMiddleTargetLocationOnField));

        /*
         * To place the Tools Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it 90 around the field's Z axis to face it away from the audience.
         * - Next we translate it back along the X axis towards the red audience wall
         * - Finally, we translate it up along the Z axis, and forward along the Y axis.
         */
        OpenGLMatrix redFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, mmFarTargetSlide, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redFarTarget.setLocation(redFarTargetLocationOnField);
//        RobotLog.ii(TAG, "Red Target=%s", format(redFarTargetLocationOnField));

       /*
        * To place the Wheels Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Then we translate it along the Y axis towards the blue audience wall.
        * - Finally, we translate it up along the Z axis, and forward along the X axis.
        */
        OpenGLMatrix blueMiddleTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmMidTargetSlide, mmFTCFieldWidth / 2, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueMidTarget.setLocation(blueMiddleTargetLocationOnField);
//        RobotLog.ii(TAG, "Blue Target=%s", format(blueMiddleTargetLocationOnField));

        /*
        * To place the Legos Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Then we translate it along the Y axis towards the blue audience wall.
        * - Finally, we translate it up along the Z axis, and forward along the X axis.
        */
        OpenGLMatrix blueFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFarTargetSlide, mmFTCFieldWidth / 2, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueFarTarget.setLocation(blueFarTargetLocationOnField);
//        RobotLog.ii(TAG, "Blue Target=%s", format(blueFarTargetLocationOnField));

        /**
         * The phone starts out lying flat, with the screen facing Up(+Z) and with the physical top of the phone
         * pointing to towards us(+Y).
         *
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing out (see our
         * choice of FRONT camera above) and in portrait mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of 90deg along the Y axis, then 90deg along
         * the X axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YXY,
                        AngleUnit.DEGREES, 90, 90, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) blueMidTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) redFarTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) redMidTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) blueFarTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        locationRelativeToTargets = new OpenGLMatrix[visionTargets.size()];
        targetsVisible = new boolean[visionTargets.size()];

        vuforiaReady = true;
    }

    public void vuforiaProcessLoop() {
        if (vuforiaReady) {
            isTargetVisible = false;
            for (int i = 0; i < visionTargets.size(); i++) {
                VuforiaTrackable trackable = visionTargets.get(i);
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                locationRelativeToTargets[i] = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                targetsVisible[i] = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();

                isTargetVisible |= ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            if (!isTargetVisible) {
                lastLocation = null;
            }
        } else {
            RobotLog.ii(TAG, "Vuforia not ready yet!");
        }
    }

    @Override
    public void runOpMode() {
        ElapsedTime vuforiaTimer = new ElapsedTime();

        this.initializeVuforia();

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.log().add("Vuforia initialized in %.2f seconds", vuforiaTimer.seconds());
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        this.activateTracking();

        while (opModeIsActive()) {
            vuforiaTimer.reset();

            this.vuforiaProcessLoop();

            for (int i = 0; i < TARGET_NAMES.length; i++) {
                telemetry.addData(TARGET_NAMES[i], this.getTargetsVisible()[i] ? "Visible" : "Not Visible");
                if (this.getLocationRelativeToTargets()[i] != null) {
                    telemetry.addData("location relative to target", Arrays.toString(extractData(this.getLocationRelativeToTargets()[i])));
                }
            }


            if (this.getLastLocation() != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(this.getLastLocation()));
                telemetry.addData("Relevant Data", Arrays.toString(extractData(this.getLastLocation())));
            } else {
                telemetry.addData("Pos", "Unknown");
                telemetry.addData("Relevant Data", "None");
            }
            telemetry.addData("Loop Time (ms)", vuforiaTimer.milliseconds());
            telemetry.update();
        }

        this.deactivateTracking();
    }
}
