package org.firstinspires.ftc.teamcode.utils.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.CONFIG;

import java.util.HashMap;
//LOTS OF WORKS NEEDS TO BE DONE HERE. BY NO MEANS IS IT DONE. YIKES!
public class Vuforia {
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    HashMap<String, VuforiaTrackable> targetDictionary = new HashMap<String, VuforiaTrackable>();
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;
    public void VuforiaSetup(){
        this.parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // This lets you see what the camera sees, drains batters you can replace with this this.parameters = new VuforiaLocalizer.Parameters();
        this.parameters.vuforiaLicenseKey = CONFIG.VuforiaKey;
        this.parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        visionTargets = vuforia.loadTrackablesFromAsset("Skystone");

    }
    public void setNames(){
        this.targetDictionary.put("TargetElement",visionTargets.get(0));
        this.targetDictionary.put("BridgeBlueRear",visionTargets.get(1));
        this.targetDictionary.put("BridgeRedRear",visionTargets.get(2));
        this.targetDictionary.put("BridgeRedFront",visionTargets.get(3));
        this.targetDictionary.put("BridgeBlueFront",visionTargets.get(4));
        this.targetDictionary.put("RedPerimeterTgt1",visionTargets.get(5));
        this.targetDictionary.put("RedPerimeterTgt2",visionTargets.get(6));
        this.targetDictionary.put("FrontPerimeterTgt1",visionTargets.get(7));
        this.targetDictionary.put("FrontPerimeterTgt2",visionTargets.get(9));
        this.targetDictionary.put("BluePerimeterTgt1",visionTargets.get(10));
        this.targetDictionary.put("BluePerimeterTgt2",visionTargets.get(11));
        this.targetDictionary.put("RearPerimeterTgt1",visionTargets.get(12));
        this.targetDictionary.put("RearPerimeterTgt2",visionTargets.get(13));
        //Take a look at FTCRobotControllet/Assets/Skystone in android view
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        //x,y,z location on the field itself
        //u,v,w are the rotations about the x, y, and z axis.
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,u,v,w));
    }
}
