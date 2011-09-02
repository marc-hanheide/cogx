/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package navigation;

/**
 *
 * @author ken
 */
public class Path {

    private int a;
    private int b;
    private double cost;

    public Path(int a, int b, double cost) {
        this.a = a;
        this.b = b;
        this.cost = cost;
    }

    public int getA() {
        return a;
    }

    public int getB() {
        return b;
    }

    public double getCost() {
        return cost;
    }
}
