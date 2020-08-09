package org.firstinspires.ftc.teamcode.utils;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import android.graphics.Color;

public class Color{
	
	ColorSensor colorSensor;
	DistanceSensor distanceSensor;
	final double SCALE_FACTOR = 255.0;
	
	public Color (HardwareMap hmap, String name){
		this.colorSensor = hmap.get(ColorSensor.class, name);
		this.distanceSensor = hardwareMap.get(DistanceSensor.class, name);
		this.colorSensor.enableLed(true);
  	}
	public double[] getRGB(){
		double[] x = {this.colorSensor.red(),this.colorSensor.green(),this.colorSensor.blue()};
		return x;
	}

	public double[] getHSV(){
		float floatHsvValues[] = {0F, 0F, 0F};
		double hsvValues[] ={0.0, 0.0, 0.0};
		Color.RGBToHSV((int) (this.colorSensor.red() * this.SCALE_FACTOR),
				(int) (this.colorSensor.green() * this.SCALE_FACTOR),
				(int) (this.colorSensor.blue() * this.SCALE_FACTOR),
				floatHsvValues);
		hsvValues = floatHsvValues;
		return hsvValues;
	}

	public double getDistanceInch(){
		return (double) this.distanceSensor.getDistance(DistanceUnit.INCH);
	}
	
	public double getDistanceCM(){
		return (double) this.distanceSensor.getDistance(DistanceUnit.CM);
	}

}
