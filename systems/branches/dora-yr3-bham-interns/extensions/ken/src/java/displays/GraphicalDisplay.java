package displays;

import java.awt.Color;
import java.awt.Graphics;
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

import util.misc;

import NavData.FNode;
import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class GraphicalDisplay extends JPanel {

	private JFrame frame;
	private Vector<PathTimes> pathTimes;
	private Vector<FNode> nodes;
	private int maxX, maxY, minX, minY;
	private int sizeX, sizeY;

	public static void main(String[] args) {

		GraphicalDisplay g = new GraphicalDisplay();

	}

	public GraphicalDisplay() {
		frame = new JFrame("map display");
		frame.setSize(800, 600);
		frame.add(this);
		load();
		findXAndY();
		System.out.println("min x is " + minX + " min y is " + minY);
		System.out.println("max x is " + maxX + " max y is " + maxY);
		System.out.println("size x is " + sizeX + " size y is " + sizeY);
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
		int height = frame.getHeight();
		int width = frame.getWidth();

		for (FNode node : nodes) {

			g.fillOval((int) (((node.x - minX) / sizeX) * width),
					(int) ((1 - (node.y - minY) / sizeY) * height), 5, 5);
			g.drawString(Long.toString(node.nodeId),
					(int) (((node.x - minX) / sizeX) * width),
					(int) ((1 - (node.y - minY) / sizeY) * height));
		}

		for (PathTimes pT : pathTimes) {
			
			g.setColor(Color.black);
			double x1 = nodes.get(pT.getA()).x;
			double y1 = nodes.get(pT.getA()).y;
			System.out.println(pT.getB());
			double x2 = nodes.get(pT.getB()).x;
			double y2 = nodes.get(pT.getB()).y;
			int drawX1 = (int) (((x1 - minX) / sizeX) * width);
			int drawY1 = (int) ((1 - ((y1 - minY) / sizeY)) * height);
			int drawX2 = (int) (((x2 - minX) / sizeX) * width);
			int drawY2 = (int) ((1 - ((y2 - minY) / sizeY)) * height);

			int n = pT.getRuns().size();

			if (n > 1) {
				double sum = 0;
				double sumSqr = 0;
				for (PathRun pR : pT.getRuns()) {
					double val = pR.timeTaken() / 1000;
					if (val != 0) {
						sum += val;
						sumSqr += (val * val);
					} else {
						sum--;
					}
				}
				double mean = sum / n;

				double var = (sumSqr - sum * mean) / (n - 1);
				// assume nothing can go over 4.5 times the var
				if (var > 4.5) {
					var = 4.5;
					System.out.println("tons of variance on this one cap'n");
				}

				g.setColor(new Color((int) ((255 / 4.5) * var),
						(int) (255 - ((255 / 4.5)) * var), 0));
			}
			g.drawLine(drawX1, drawY1, drawX2, drawY2);

		}

	}

}
