package org.firstinspires.ftc.teamcode.utils;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class ColorSensors {
	
	ColorSensor colorSensor;
	DistanceSensor distanceSensor;
	final double SCALE_FACTOR = 255.0;
	
	public ColorSensors(HardwareMap hmap, String name){
		this.colorSensor = hmap.get(ColorSensor.class, name);
		this.distanceSensor = hmap.get(DistanceSensor.class, name);
		this.colorSensor.enableLed(true);

  	}
	public double[] getRGB(){
		double[] x = {this.colorSensor.red(),this.colorSensor.green(),this.colorSensor.blue()};
		return x;
	}

	public double[] getHSV(){
		float[] floatHsvValues = {0F, 0F, 0F};
		double hsvValues[] = new double[3];
		Color.RGBToHSV((int) (this.colorSensor.red() * this.SCALE_FACTOR),
				(int) (this.colorSensor.green() * this.SCALE_FACTOR),
				(int) (this.colorSensor.blue() * this.SCALE_FACTOR),
				floatHsvValues);
		for(int i=0;i<floatHsvValues.length;i++){
			hsvValues[i] = (double) hsvValues[i];
		}
		return hsvValues;
	}

	public double getDistanceInch(){
		double distance =distanceSensor.getDistance(DistanceUnit.INCH);
		return distance;
	}
	
	public double getDistanceCM(){
		return (double) this.distanceSensor.getDistance(DistanceUnit.CM);
	}

}
