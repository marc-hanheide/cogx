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
public class AStarSearch {

    

    public Path getPath(int a, int b, Vector<Path> paths) {
        for (Path path : paths) {
            if ((path.getA() == a && path.getB() == b) || (path.getA() == b && path.getB() == a)) {
                return path;
            }
        }
        return null;
    }

    public static Vector<Integer> search(Vector<FNode> nodes, Vector<Path> paths, int startPoint, int endPoint) {
        Vector<PathsFromNode> PFN = PathsFromNode.getPathsFromNodes(paths, nodes);
        Vector<Integer> nodesVisited = new Vector<Integer>();
        Vector<AStarNode> toVisit = new Vector<AStarNode>();
        Vector<AStarNode> pathsVisited = new Vector<AStarNode>();

        nodesVisited.add(new Integer(startPoint));

        for (Path path : PFN.get(startPoint).getPaths()) {
            toVisit.add(new AStarNode(0, path, nodes.get(path.getA()), nodes.get(path.getB()), nodes.get(endPoint)));
        }

        boolean found = false;
        while (!found) {
            //find next best pos
            double best = Double.MAX_VALUE;
            int bestPos = 0;
            for (int i = 0; i < toVisit.size(); i++) {
                if (toVisit.get(i).getCost() < best) {
                    bestPos = i;
                    best = toVisit.get(i).getCost();
                }
            }

            int pos = toVisit.get(bestPos).getNode();
            pathsVisited.add(toVisit.remove(bestPos));
            Vector<Path> temp = PFN.get(pos).getPaths();
            for (Path path : temp) {
                if (path.getA() == pos) {
                    if (!nodesVisited.contains(path.getB())) {
                        toVisit.add(new AStarNode(best, path, nodes.get(pathsVisited.get(pathsVisited.size()-1).getPrevNode()),
                                nodes.get(path.getA()), nodes.get(path.getB()), nodes.get(endPoint)));
                        nodesVisited.add(new Integer(path.getA()));
                        if (path.getB() == endPoint) {
                            found = true;
                            pathsVisited.add(toVisit.remove(toVisit.size() - 1));
                        }
                    }
                } else {
                    if (!nodesVisited.contains(path.getA())) {
                        toVisit.add(new AStarNode(best, path,nodes.get(pathsVisited.get(pathsVisited.size()-1).getPrevNode()),
                                nodes.get(path.getB()), nodes.get(path.getA()), nodes.get(endPoint)));
                        nodesVisited.add(new Integer(path.getB()));
                        if (path.getA() == endPoint) {
                            found = true;
                            pathsVisited.add(toVisit.remove(toVisit.size()));
                        }
                    }
                }
            }



        }


        //go through pathsVisited to find each path
        int pos = endPoint;
        Vector<Integer> path = new Vector<Integer>();
        path.add(new Integer(endPoint));
        while (pos != startPoint) {
            for (AStarNode node : pathsVisited) {
                if (node.getNode() == pos) {
                    pos = node.getPrevNode();
                    path.add(0, new Integer(node.getPrevNode()));

                }
            }
        }


        return path;

    }

    
}
