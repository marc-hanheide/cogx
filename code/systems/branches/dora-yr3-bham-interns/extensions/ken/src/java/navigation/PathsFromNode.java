/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package navigation;

import java.util.Vector;


import NavData.FNode;

/**
 *
 * @author ken
 */
public class PathsFromNode {

    private Vector<Path> paths;
    private FNode node;

    public PathsFromNode(FNode node) {
       this.node=node;
       paths = new Vector<Path>();
    }

    public static Vector<PathsFromNode> getPathsFromNodes(Vector<Path> pathList, Vector<FNode> nodes) {
        Vector<PathsFromNode> PFN = new Vector<PathsFromNode>();
        for (FNode node : nodes) {
            PFN.add(new PathsFromNode(node));
        }
        for (Path path : pathList) {
            PFN.get(path.getA()).addPath(path);
            PFN.get(path.getB()).addPath(path);
        }
        return PFN;
    }

    public void addPath(Path path) {
       
        paths.add(path);
    }

    public FNode getNode() {
        return node;
    }

  
    public Vector<Path> getPaths() {
        return paths;
    }
}
