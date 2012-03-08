package displays;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.JPanel;

import NavData.FNode;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class GraphicalDisplay extends JPanel {

	private JFrame frame;
	private Vector<PathTimes> pathTimes;
	private Vector<FNode> nodes;
	private int maxX, maxY, minX, minY;
	private int sizeX, sizeY;
	private Vector<Integer> path;

	public static void main(String[] args) {

		GraphicalDisplay g = new GraphicalDisplay(new Vector<Integer>(), "map display");

	}

	public GraphicalDisplay(Vector<Integer> path, String title) {
		frame = new JFrame(title);
		this.path = path;
		frame.setSize(800, 600);
		frame.add(this);
		load();
		findXAndY();
//		System.out.println("min x is " + minX + " min y is " + minY);
//		System.out.println("max x is " + maxX + " max y is " + maxY);
//		System.out.println("size x is " + sizeX + " size y is " + sizeY);
		frame.setVisible(true);
	}

	
	
	
	public void findXAndY() {
		double maxX = Double.MIN_VALUE;
		double maxY = Double.MIN_VALUE;
		double minX = Double.MAX_VALUE;
		double minY = Double.MAX_VALUE;

		for (FNode node : nodes) {
			if (node.x > maxX) {
				maxX = node.x;
			} else {
				if (node.x < minX) {
					minX = node.x;
				}
			}
			if (node.y > maxY) {
				maxY = node.y;
			} else {
				if (node.y < minY) {
					minY = node.y;
				}
			}
		}
		this.maxX = (int) maxX + 1;
		this.maxY = (int) maxY + 1;
		this.minX = (int) minX - 1;
		this.minY = (int) minY - 1;
		sizeX = this.maxX - this.minX;
		sizeY = this.maxY - this.minY;
	}

	public void load() {

		pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();

			BufferedReader f = new BufferedReader(
					new FileReader("tmpmap.graph"));
			String first = f.readLine();
			int length = Integer.valueOf(String.valueOf(first.toCharArray(), 6,
					first.length() - 6));

			nodes = new Vector<FNode>();

			while (nodes.size() < length) {
				nodes.add(util.misc.string2FNode(f.readLine()));

			}
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}

	}

	@Override
	public void paintComponent(Graphics g) {
		Graphics2D g2 = (Graphics2D)g;
		g2.setStroke(new BasicStroke(2));
		int height = frame.getHeight();
		int width = frame.getWidth();

		for (FNode node : nodes) {

			g.fillOval((int) (((node.x - minX) / sizeX) * width),
					(int) ((1 - (node.y - minY) / sizeY) * height), 5, 5);
			g.drawString(Long.toString(node.nodeId),
					(int) (((node.x - minX) / sizeX) * width),
					(int) ((1 - (node.y - minY) / sizeY) * height));
		}

		for (int i = 0; i < pathTimes.size(); i++) {
			PathTimes pT = pathTimes.get(i);

			double x1 = nodes.get(pT.getA()).x;
			double y1 = nodes.get(pT.getA()).y;
			//System.out.println(pT.getB());
			double x2 = nodes.get(pT.getB()).x;
			double y2 = nodes.get(pT.getB()).y;
			int drawX1 = (int) (((x1 - minX) / sizeX) * width);
			int drawY1 = (int) ((1 - ((y1 - minY) / sizeY)) * height);
			int drawX2 = (int) (((x2 - minX) / sizeX) * width);
			int drawY2 = (int) ((1 - ((y2 - minY) / sizeY)) * height);
			if (containsPath(pT.getA(),pT.getB())) {
				g.setColor(Color.green);
			} else {
				g.setColor(Color.black);
			}

			g.drawLine(drawX1, drawY1, drawX2, drawY2);
		}

	}
	
	
	public boolean containsPath(int a, int b){
		
		for(int i=1;i<path.size();i++){
			if(path.get(i)==a && path.get(i-1)==b){
				return true;
			}else{
				
				if(path.get(i)==b && path.get(i-1)==a){
					return true;
				}
			}
		}
		return false;
	}
	
}
