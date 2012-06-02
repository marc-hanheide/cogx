package cdsr.data;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * 
 * @author Graham Horn
 *
 */
public class OccupancyMap
{

  private final int[][] m_occupancy;
  private final double m_minX;
  private final double m_minY;
  private final double m_sampleSize;

  public OccupancyMap(int size_i, int size_j, double min_x, double min_y,
      double sample_size)
  {
    m_occupancy = new int[size_i][size_j];
    for (int i = 0; i < size_i; i++)
    {
      for (int j = 0; j < size_j; j++)
      {
        m_occupancy[i][j] = 0;
      }
    }
    m_minX = min_x;
    m_minY = min_y;
    m_sampleSize = sample_size;
  }

  public OccupancyMap(int[][] _occupancy, double min_x, double min_y,
      double sample_size)
  {
    m_occupancy = _occupancy;
    m_minX = min_x;
    m_minY = min_y;
    m_sampleSize = sample_size;
  }

  public int[][] getOccupancy()
  {
    return m_occupancy;
  }

  public void addOccupancy(int i, int j)
  {
    m_occupancy[i][j]++;
  }

  public String toString()
  {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < m_occupancy.length; i++)
    {
      for (int j = 0; j < m_occupancy[i].length; j++)
      {
        sb.append(m_occupancy[i][j]);
        sb.append(" ");
      }
      sb.append("\n");
    }
    return sb.toString();
  }

  public void writeToFile(String _filename)
  {
    File file = new File(_filename);

    if (file.exists())
    {
      boolean deleted = file.delete();
      if (!deleted)
      {
        throw new RuntimeException("Could not delete file before writing: "
            + _filename);
      }
    }

    try
    {
      BufferedWriter bw = new BufferedWriter(new FileWriter(file));
      bw.write(m_sampleSize + "\n");
      bw.write(m_minX + " " + m_minY + "\n");
      bw.write(m_occupancy.length + " " + m_occupancy[0].length + "\n");
      bw.write(this.toString());
      bw.flush();
      bw.close();
    }
    catch (IOException e)
    {
      e.printStackTrace();
    }

  }

}
