package frc.robot.utils;

public class Library {

   private double prevGap = 99.0;

   public boolean isMoving(double pos, double sp) {
      double currGap = Math.abs(pos - sp);
      boolean moving = currGap < prevGap;
      System.out.println(
            "pos/sp: " + pos + "/" + sp +
                  " prevGap: " + prevGap + " currGap: " + currGap +
                  " Moving: " + moving);
      prevGap = currGap;
      return moving;
   }
}
