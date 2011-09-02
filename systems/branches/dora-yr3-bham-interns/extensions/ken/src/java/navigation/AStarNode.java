/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package navigation;

import java.awt.geom.Line2D;
import java.util.Vector;


import NavData.FNode;

/**
 *
 * @author ken
 */
public class AStarNode {

    private int nodeId;
    private double cost;
    private int prevNode;
    private static long turnTime;//the amount of time taken to turn 1 rad

    public AStarNode(double costSoFar, Path path, FNode precursorNode, FNode prevNode, FNode node, FNode endNode) {
        nodeId = (int) node.nodeId;
        double turnCost = turnTime*requiredHeading(prevNode, precursorNode, node);
        cost = costSoFar + turnCost + path.getCost() + distance(node, endNode); // f(x) = costSoFar + paths's cost + distance to end
        this.prevNode = (int) prevNode.nodeId;
    }

    public AStarNode(double costSoFar, Path path, FNode prevNode, FNode node, FNode endNode) {
        nodeId = (int) node.nodeId;

        cost = costSoFar + path.getCost() + distance(node, endNode); // f(x) = costSoFar + paths's cost + distance to end
        this.prevNode = (int) prevNode.nodeId;
    }

    /**
     * given the time taken to do a complete turn,
     * sets how long it takes to turn 1 rad
     * @param time2Turn 
     */
    public static void setWholeTurn(long time2Turn) {
        turnTime = (long)(time2Turn/(2*Math.PI));
    }

    private static double distance(FNode prevNode, FNode nextNode) {
        double xComp = Math.pow(prevNode.x - nextNode.x, 2);
        double yComp = Math.pow(prevNode.y - nextNode.y, 2);
        return Math.sqrt(xComp + yComp);
    }

    public int getNode() {
        return nodeId;
    }

    public double getCost() {
        return cost;
    }

    public int getPrevNode() {
        return prevNode;
    }

    @Override
    public String toString() {
        return "AStar node from " + prevNode + " to " + nodeId + " cost " + cost;
    }

    /**
     * given the index of a FNode, compute the heading from the current FNode
     * 
     * @param index
     * @return
     */
    public double requiredHeading(int x0, int y0, int x1, int y1, int x2, int y2) {

        Line2D.Double a = new Line2D.Double(x0, y0, x1, y1);
        Line2D.Double b = new Line2D.Double(x0, y0, x2, y2);
        return angleBetween2Lines(a, b);

    }

    public double requiredHeading(FNode centre, FNode p1, FNode p2) {
        double temp = Math.abs(requiredHeading((int) centre.x, (int) centre.y, (int) p2.x, (int) p2.y, (int) p1.x, (int) p1.y));

        temp = Math.PI - temp;
        if (temp >= Math.PI) {
            temp -= Math.PI;
        }
        System.out.println("centre " + centre.nodeId);
        System.out.println("prec " + p1.nodeId);
        System.out.println("post " + p2.nodeId);
        System.out.println("turn amount " + temp);
        return temp;
    }

    /**
     * gives the angle between two lines
     * 
     * from http://stackoverflow.com/questions/3365171/calculating-the-angle-
     * between-two-lines-without-having-to-calculate-the-slope-j/3366569#3366569
     * 
     * @param line1
     * @param line2
     * @return
     */
    private double angleBetween2Lines(Line2D line1, Line2D line2) {
        double angle1 = Math.atan2(line1.getY1() - line1.getY2(), line1.getX1()
                - line1.getX2());
        double angle2 = Math.atan2(line2.getY1() - line2.getY2(), line2.getX1()
                - line2.getX2());
        return angle1 - angle2;
    }
}
