package manipulation.core.share.types;

/**
 * represents a map of the environment
 * 
 * @author ttoenige
 * 
 */
public class GlobalMap {
	// TODO variabler machen, sollte auch fuer bi funktionieren
	// TODO globalen Typ machen nicht char[]?!
	private char[] data;
	private int size;
	private int xSize;
	private int ySize;
	private Vector2D center;
	private double cellSize;

	/**
	 * constructor of a global map
	 * 
	 * @param data
	 *            relevant map data
	 * @param size
	 *            size of the map (map is square, xSize = ySize = (2 * size +
	 *            1))
	 * @param center
	 *            center of the map
	 * @param cellSize
	 *            size of a cell map
	 */
	public GlobalMap(char[] data, int size, Vector2D center, double cellSize) {
		this.data = data;
		this.size = size;
		this.center = center;
		this.cellSize = cellSize;

		this.xSize = (2 * size + 1);
		this.ySize = (2 * size + 1);
	}

	/**
	 * empty constructor of a map
	 */
	public GlobalMap() {

	}

	/**
	 * gets the data of the map
	 * 
	 * @return data of the map
	 */
	public char[] getData() {
		return data;
	}

	/**
	 * sets the corresponding map data
	 * 
	 * @param data
	 *            relevant data to be set
	 */
	public void setData(char[] data) {
		this.data = data;
	}

	/**
	 * gets the size of the map (map is square, xSize = ySize = (2 * size + 1))
	 * 
	 * @return size of the map (map is square, xSize = ySize = (2 * size + 1))
	 */
	public int getSize() {
		return size;
	}

	/**
	 * sets the size of the map
	 * 
	 * @param size
	 *            relevant new size value
	 */
	public void setSize(int size) {
		this.size = size;
		this.xSize = (2 * size + 1);
		this.ySize = (2 * size + 1);
	}

	/**
	 * gets the center point of the map
	 * 
	 * @return center point of the map
	 */
	public Vector2D getCenter() {
		return center;
	}

	/**
	 * sets the center of the map
	 * 
	 * @param center
	 *            relevant center point
	 */
	public void setCenter(Vector2D center) {
		this.center = center;
	}

	/**
	 * gets the cell size of the map
	 * 
	 * @return cell size of the map
	 */
	public double getCellSize() {
		return cellSize;
	}

	/**
	 * sets the cell size of the map
	 * 
	 * @param cellSize
	 *            new cell size value
	 */
	public void setCellSize(double cellSize) {
		this.cellSize = cellSize;
	}

	/**
	 * gets the x-size of the map (map is square)
	 * 
	 * @return x-size of the map (map is square)
	 */
	public int getxSize() {
		return xSize;
	}

	/**
	 * gets the y-size of the map (map is square)
	 * 
	 * @return y-size of the map (map is square)
	 */
	public int getySize() {
		return ySize;
	}
}
