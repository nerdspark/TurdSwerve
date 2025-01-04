package frc.robot.utils;

public class PointDW {

    public double x, y;

    public PointDW(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public PointDW() {
        this(0, 0);
    }

    public PointDW(double[] vals) {
        this();
        set(vals);
    }

    public void set(double[] vals) {
        if (vals != null) {
            x = vals.length > 0 ? vals[0] : 0;
            y = vals.length > 1 ? vals[1] : 0;
        } else {
            x = 0;
            y = 0;
        }
    }

    public PointDW clone() {
        return new PointDW(x, y);
    }

    public double dot(PointDW p) {
        return x * p.x + y * p.y;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof PointDW)) return false;
        PointDW it = (PointDW) obj;
        return x == it.x && y == it.y;
    }


    @Override
    public String toString() {
        return "{" + x + ", " + y + "}";
    }
}
