package frc.utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class BatteryVoltageSim {
    private static BatteryVoltageSim instance;

    private double voltage = 11.5;//unloaded voltage
    private double nominalcurrent = 20.0;//default current draw
    private double current = 0.0;

    private ArrayList<DoubleSupplier> currentSources = new ArrayList<>();

    private BatteryVoltageSim(){
        currentSources.add(()->nominalcurrent);
    }
    public static synchronized BatteryVoltageSim getInstance(){
        if(instance == null){
            instance = new BatteryVoltageSim();
        }
        return instance;
    }

    public void addCurrentSource(DoubleSupplier currentSource){
        currentSources.add(currentSource);
    }
    
    public double calculateVoltage(){
        current = 0.0;
        for(DoubleSupplier currentSource : currentSources){
            current += currentSource.getAsDouble();
        }

        double Loadvoltage = BatterySim.calculateLoadedBatteryVoltage(voltage, 0.015, current);
        RoboRioSim.setVInVoltage(voltage);
        return Loadvoltage;
    }    

}
