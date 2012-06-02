package cdsr.data;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import cdsr.objects.AnnotatedCDSR;
import cdsr.objects.Room;

/**
 * Generates occupancy maps for each subject and an overall heat map for the
 * room.
 * 
 * @author Graham Horn
 * 
 */
public class OccupancyDataGenerator
{

  public double m_sampleSize;

  private Map<String, OccupancyMap> m_subjectToOccupancy;
  private OccupancyMap m_heatMap;
  private Set<AnnotatedCDSR> m_annotatedCdsrs;

  public OccupancyDataGenerator(Room _room, Set<AnnotatedCDSR> annotated_cdsrs,
      double sample_size)
  {
    RoomProcessor roomProcessor = new RoomProcessor(_room);
    m_sampleSize = sample_size;
    m_annotatedCdsrs = annotated_cdsrs;
    double min_x = roomProcessor.getBottomLeft().getX();
    double max_x = roomProcessor.getBottomRight().getX();
    double min_y = roomProcessor.getBottomLeft().getY();
    double max_y = roomProcessor.getTopLeft().getY();

    double sideLengthX = max_x - min_x;
    double sideLengthY = max_y - min_y;

    System.out.println(sideLengthX + " " + sideLengthY);

    int numSamplesX = (int) Math.round(sideLengthX / m_sampleSize);
    int numSamplesY = (int) Math.round(sideLengthY / m_sampleSize);

    m_heatMap = new OccupancyMap(numSamplesX, numSamplesY, min_x, min_y,
        m_sampleSize);

    Iterator<AnnotatedCDSR> itr = annotated_cdsrs.iterator();

    m_subjectToOccupancy = new HashMap<String, OccupancyMap>();

    while (itr.hasNext())
    {
      AnnotatedCDSR annotated_cdsr = itr.next();
      Polygon polygon = new Polygon();

      List<Line2D.Double> lines = annotated_cdsr.getLines();
      polygon.addPoint(
          (int) Math.round((lines.get(0).getX1() - min_x) / m_sampleSize),
          (int) Math.round((lines.get(0).getY1() - min_y) / m_sampleSize));
      for (int i = 0; i < lines.size(); i++)
      {
        polygon.addPoint(
            (int) Math.round((lines.get(i).getX2() - min_x) / m_sampleSize),
            (int) Math.round((lines.get(i).getY2() - min_y) / m_sampleSize));
      }

      OccupancyMap occupancy_map = new OccupancyMap(numSamplesX, numSamplesY,
          min_x, min_y, m_sampleSize);

      for (int i = 0; i < numSamplesX; i++)
      {
        for (int j = 0; j < numSamplesY; j++)
        {
          if (polygon.contains(i, j))
          {
            occupancy_map.addOccupancy(i, j);
            m_heatMap.addOccupancy(i, j);
          }
        }
      }

      m_subjectToOccupancy.put(annotated_cdsr.getSubject(), occupancy_map);
    }

  }

  public OccupancyMap getHeatMap()
  {
    return m_heatMap;
  }

  public Map<String, OccupancyMap> getSubjectToOccupancy()
  {
    return m_subjectToOccupancy;
  }

  public void writeDataToFiles(String prefix)
  {
    String room = m_annotatedCdsrs.iterator().next().getRoom();

    m_heatMap.writeToFile(prefix + "/" + room + "_heatmap.txt");
    for (Iterator<String> iterator = m_subjectToOccupancy.keySet().iterator(); iterator
        .hasNext();)
    {
      String subject = iterator.next();
      m_subjectToOccupancy.get(subject).writeToFile(
          prefix + "/" + room + "_" + subject + ".txt");
    }
  }

  public static void main(String[] _args) throws IOException,
      ClassNotFoundException
  {
    List<Line2D.Double> lines = new ArrayList<Line2D.Double>();
    // test data:
    // lines.add(new Line2D.Double(1.1, 1.2, 1.3, 3.2));
    // lines.add(new Line2D.Double(1.3, 3.2, 3.3, 3.2));
    // lines.add(new Line2D.Double(3.3, 3.2, 3.4, 1.1));
    // lines.add(new Line2D.Double(3.4, 1.1, 1.1, 1.2));
    // AnnotatedCDSR annotated_cdsr = new AnnotatedCDSR("Graham", "Classroom1",
    // "file1", lines, "front", new Point2D.Double(-0.3125, 0.075));

    // -----------------------------------------
    // translated subject data
    // The first line seems to be disconnected from the others; ignoring it
    // lines.add(new Line2D.Double(-0.9, 1.4, -0.775, 1.35));
    lines.add(new Line2D.Double(-0.8375, 0.875, -0.875, -0.9));
    lines.add(new Line2D.Double(-0.875, -0.9, 0.275, -0.925));
    lines.add(new Line2D.Double(0.275, -0.9, 0.275, 0.9));
    lines.add(new Line2D.Double(0.275, 0.9, -0.85, 0.8875));
    AnnotatedCDSR annotated_cdsr1 = new AnnotatedCDSR("subject-1",
        "classroom-222-classroom-format-real",
        "data/classroom-222-classroom-format-real.cdsr", lines, "front",
        new Point2D.Double(-0.3125, 0.075));

    List<Line2D.Double> lines5 = new ArrayList<Line2D.Double>();
    lines5.add(new Line2D.Double(-0.975, 2.2, 0.675, 2.1625));
    lines5.add(new Line2D.Double(0.675, 2.1625, 0.725, -1.85));
    lines5.add(new Line2D.Double(0.725, -1.85, -1.2375, -1.75));
    lines5.add(new Line2D.Double(-1.2375, -1.75, -0.9875, 2.225));
    AnnotatedCDSR annotated_cdsr5 = new AnnotatedCDSR("subject-5",
        "classroom-222-classroom-format-real",
        "data/classroom-222-classroom-format-real.cdsr", lines5, "front",
        new Point2D.Double(-0.3625, 0.2625));

    List<Line2D.Double> lines6 = new ArrayList<Line2D.Double>();
    lines6.add(new Line2D.Double(-0.975, 2.2, -1.0375, -1.675));
    lines6.add(new Line2D.Double(-1.0375, -1.675, 0.6875, -1.7875));
    lines6.add(new Line2D.Double(0.6875, -1.7875, 0.8375, 2.1875));
    lines6.add(new Line2D.Double(0.8375, 2.1875, -1.0125, 2.1875));
    AnnotatedCDSR annotated_cdsr6 = new AnnotatedCDSR("subject-6",
        "classroom-222-classroom-format-real",
        "data/classroom-222-classroom-format-real.cdsr", lines6, "front",
        new Point2D.Double(0.025, 0.1375));

    ArrayList<Line2D.Double> walls = new ArrayList<Line2D.Double>();
    walls.add(new Line2D.Double(-1, -2.3, -1, 2.3));
    walls.add(new Line2D.Double(-1.3, -2, 4, -2));
    Room room = new Room("Classroom1", walls);

    Set<AnnotatedCDSR> annotated_cdsrs = new HashSet<AnnotatedCDSR>();
    annotated_cdsrs.add(annotated_cdsr1);
    annotated_cdsrs.add(annotated_cdsr5);
    annotated_cdsrs.add(annotated_cdsr6);

    // ProblemSet ps = CDSRMarshaller.loadProblemSet(_args[0]);

    OccupancyDataGenerator generator = new OccupancyDataGenerator(room,
        annotated_cdsrs, 0.1);

    System.out.println(generator.getHeatMap());
    generator.writeDataToFiles(_args[0]);
  }
}
