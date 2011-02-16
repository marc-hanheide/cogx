package de.dfki.lt.tr.dialogue.cplan;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;

/** A bipartite undirected graph with n nodes on one side and m nodes on the
 *  other.
 *  When all edges have been added, the maximum matching can be computed using
 *  the Hopcroft-Karp algorithm.
 *
 *  Original C++ source code thanks to Zobayer Hasan:
 *  http://zobayer.blogspot.com/2010/05/maximum-matching.html
 */
public class BipartiteGraph {

  static final int NIL = 0;
  static final int INF = Integer.MAX_VALUE;

  private Vector<Integer>[] G;
  private int n;
  private int[] match, dist;
  // n: number of nodes on left side, nodes are numbered 1 to n
  // m: number of nodes on right side, nodes are numbered n+1 to n+m
  // |G| = m + n + 1
  // G = NIL[0] U G1[G[1---n]] U G2[G[n+1---n+m]]

  @SuppressWarnings("unchecked")
  public BipartiteGraph(int n, int m) {
    this.G = new Vector[n + 1];
    for (int i = 0; i < G.length; ++i) {
      G[i] = new Vector<Integer>();
    }
    this.n = n;
    match = new int[n + m + 1];
    dist = new int[n + m + 1];
    for (int i = 0; i < match.length; ++i) {
      match[i] = NIL;
      dist[i] = 0;
    }
  }

  /** add an undirected edge to the bipartite graph */
  public void add(int from, int to) {
    G[from + 1].add(to + n + 1);
  }

  @Override
  public String toString() {
    return printGraph() + printData();
  }

  private String printData() {
    StringBuffer sb = new StringBuffer();
    sb.append("Match:");
    for (int i : match) {
      sb.append(" ").append(i);
    }
    sb.append("\nDist :");
    for (int i : dist) {
      sb.append(" ").append(i);
    }
    return sb.toString();
  }

  private String printGraph() {
    StringBuffer sb = new StringBuffer();
    int v = 0;
    for (Vector<Integer> edges : G) {
      sb.append(v++).append(':');
      for (int t : edges) {
        sb.append(' ').append(t);
      }
      sb.append('\n');
    }
    return sb.toString();
  }

  private Boolean bfs() {
    Queue< Integer > Q = new LinkedList<Integer>();
    for(int i = 1; i <= n; i++) {
      if(match[i] == NIL) {
        dist[i] = 0;
        Q.add(i);
      }
      else dist[i] = INF;
    }
    dist[NIL] = INF;
    while(!Q.isEmpty()) {
      int u = Q.poll();
      if(u != NIL) {
        for(int v: G[u]) {
          if(dist[match[v]] == INF) {
            dist[match[v]] = dist[u] + 1;
            Q.add(match[v]);
          }
        }
      }
    }
    return (dist[NIL] != INF);
  }

  private Boolean dfs(int u) {
    if(u != NIL) {
      for(int v : G[u]) {
        if(dist[match[v]] == dist[u] + 1) {
          if(dfs(match[v])) {
            match[v] = u;
            match[u] = v;
            return true;
          }
        }
      }
      dist[u] = INF;
      return false;
    }
    return true;
  }

  public int hopcroft_karp() {
    int matching = 0;
    //System.out.println(printGraph());
    // match[] is assumed NIL for all vertex in G
    while(bfs()) {
      //System.out.println(printData());
      for(int i = 1; i <= n; i++) {
        if (match[i] == NIL && dfs(i)) {
          //System.out.println(printData());
          matching++;
        }
      }
    }
    return matching;
  }

  public int[] getMatch() {
    return match;
  }
}
