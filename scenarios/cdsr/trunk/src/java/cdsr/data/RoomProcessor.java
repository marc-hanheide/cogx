package cdsr.data;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Iterator;

import cdsr.objects.Room;

/**
 * Find the corners of the room using the smallest and largest x and y
 * coordinates of all of the wall segments.
 * 
 * @author GSH
 * 
 */
public class RoomProcessor
{

  private Room m_room;
  private Point2D.Double m_topLeft;
  private Point2D.Double m_topRight;
  private Point2D.Double m_bottomLeft;
  private Point2D.Double m_bottomRight;

  public RoomProcessor(Room _room)
  {
    m_room = _room;
    processRoom();
  }

  public Room getRoom()
  {
    return m_room;
  }

  public Point2D.Double getTopLeft()
  {
    return m_topLeft;
  }

  public Point2D.Double getTopRight()
  {
    return m_topRight;
  }

  public Point2D.Double getBottomLeft()
  {
    return m_bottomLeft;
  }

  public Point2D.Double getBottomRight()
  {
    return m_bottomRight;
  }

  /**
   * Naively assumes that the room is orientated along the x-y axis also does
   * not discard bad line segments so the room corners will give a much bigger
   * room than is actually the case.
   */
  private void processRoom()
  {
    double max_x = Double.MIN_VALUE;
    double min_x = Double.MAX_VALUE;
    double max_y = Double.MIN_VALUE;
    double min_y = Double.MAX_VALUE;

    Iterator<Line2D.Double> itr = m_room.iterator();
    while (itr.hasNext())
    {
      Line2D.Double next_line = (Line2D.Double) itr.next();
      if (next_line.getX1() < min_x)
      {
        min_x = next_line.getX1();
      }
      if (next_line.getX2() < min_x)
      {
        min_x = next_line.getX2();
      }
      
      if (next_line.getX1() > max_x)
      {
        max_x = next_line.getX1();
      }
      if (next_line.getX2() > max_x)
      {
        max_x = next_line.getX2();
      }

      if (next_line.getY1() < min_y)
      {
        min_y = next_line.getY1();
      }
      if (next_line.getY2() < min_y)
      {
        min_y = next_line.getY2();
      }
      
      if (next_line.getY1() > max_y)
      {
        max_y = next_line.getY1();
      }
      if (next_line.getY2() > max_y)
      {
        max_y = next_line.getY2();
      }
    }

    m_topLeft = new Point2D.Double(min_x, max_y);
    m_bottomLeft = new Point2D.Double(min_x, min_y);
    m_topRight = new Point2D.Double(max_x, max_y);
    m_bottomRight = new Point2D.Double(max_x, min_y);

  }
}
