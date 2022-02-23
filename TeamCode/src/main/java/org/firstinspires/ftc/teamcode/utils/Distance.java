package org.firstinspires.ftc.teamcode.utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Distance{

	DistanceSensor distanceSensor;

	public Distance (HardwareMap hmap, String name){
		this.distanceSensor = hmap.get(DistanceSensor.class, name);
	}

	public double getDistance(){
		return getDistanceIn();
	}

	public double getDistanceIn(){
		return (double) this.distanceSensor.getDistance(DistanceUnit.INCH);
	}

	public double getDistanceCm(){
		return (double) this.distanceSensor.getDistance(DistanceUnit.CM);
	}

}
