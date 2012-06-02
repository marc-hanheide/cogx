package cdsr.data;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import cdsr.objects.AnnotatedCDSR;

/**
 * This class is used to read the results for a single subject.
 * 
 * @author Graham Horn
 * 
 */
public class SubjectDataReader
{

  public static Set<AnnotatedCDSR> processSubjectDataFile(
      String subject_data_file) throws IOException
  {
    HashSet<AnnotatedCDSR> annotatedCdrs = new HashSet<AnnotatedCDSR>();
    BufferedReader reader = new BufferedReader(
        new FileReader(subject_data_file));
    int firstHyphen = subject_data_file.indexOf('-');
    int firstPeriod = subject_data_file.indexOf('.');
    String subject = subject_data_file.substring(firstHyphen + 1, firstPeriod);
    // System.out.println("subject name = " + subject);
    String next_line;
    while ((next_line = reader.readLine()) != null)
    {
      AnnotatedCDSR annotatedCdsr = processLine(subject, next_line);
      // System.out.println(annotatedCdsr);
      annotatedCdrs.add(annotatedCdsr);
    }
    return annotatedCdrs;
  }

  protected static AnnotatedCDSR createAnnotatedCDSRFromDataString(
      String _subject, String _room, String _cdsrDataFile, String _data)
  {
    String[] split = _data.split("[ ()]+");
    ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>();

    for (int i = 0; i < split.length - 3; i += 4)
    {
      try
      {
        lines.add(new Line2D.Double(Double.parseDouble(split[i]), Double
            .parseDouble(split[i + 1]), Double.parseDouble(split[i + 2]),
            Double.parseDouble(split[i + 3])));
      }
      catch (NumberFormatException e)
      {
        System.out.println("Number format exception");
      }
    }

    Point2D.Double sweetspot = null;
    try
    {
      sweetspot = new Point2D.Double(
          Double.parseDouble(split[split.length - 3]),
          Double.parseDouble(split[split.length - 2]));
    }
    catch (NumberFormatException e)
    {
      System.out.println("Number format exception");
      // HACK this not really a good way of solving this problem
    }

    String type = split[split.length - 1];

    return new AnnotatedCDSR(_subject, _room, _cdsrDataFile, lines, type,
        sweetspot);
  }

  protected static AnnotatedCDSR processLine(String _subject, String line)
  {
    int firstSpace = line.indexOf(' ');
    String cdsrDataFile = line.substring(0, firstSpace);
    String[] splitFilepath = cdsrDataFile.split("/");
    String filename = splitFilepath[splitFilepath.length - 1];
    // TODO hardcoded length of file suffix .cdsr
    String room = filename.substring(0, filename.length() - 5);

    String groundTruthData = line.substring(firstSpace + 1);
    return createAnnotatedCDSRFromDataString(_subject, room, cdsrDataFile,
        groundTruthData);

  }

  public static void main(String[] _args)
  {
    try
    {
      System.out.println(processSubjectDataFile(_args[0]));
    }
    catch (IOException e)
    {
      e.printStackTrace();
    }
  }
}
