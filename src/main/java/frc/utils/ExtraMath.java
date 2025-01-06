package frc.utils;

import edu.wpi.first.math.MathUtil;

public final class ExtraMath {
    
  /**
   * process input value
   * 
   * curve function graphed here {@link https://www.desmos.com/calculator/fjuc4iqjqt}
   * @param val - number to process
   * @param multiplier - multiplier for input, mainly used for inverting
   * @param square - polynomial curve value, roughly y=x^s {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs}
   * swerve subsystem drive commands square internaly, so this should not be used
   * @param deadZone - deadzone for input {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#input-deadband}
   * @return
   */
  public static double processInput(Double val, Double multiplier, Double square, Double deadZone){
    double out = val;

    if(square     != null){
      if(deadZone != null && deadZone > 0){
        out  = Math.signum(val) * Math.pow((1/(-deadZone+1))*Math.abs(val)-(deadZone/(-deadZone+1)), square);
      } else {
        out  = Math.signum(val) * Math.pow(Math.abs(val), square);
      }
    }
    if(deadZone   != null){out  = MathUtil.applyDeadband(out, deadZone);}
    if(multiplier != null){out *= multiplier;}

    return out;
  }

  public static double remap(double val, double inMin, double inMax, double outMin, double outMax){
    return ((val-inMin)/(inMax-inMin)*(outMax-outMin))+outMin;
  }

  public static double lerp(double start, double end, double val){
    return (end-start)*val + start;
  }

  public static double holdPositive(double in){
    return in<0 ? 0 : in;
  }
}
