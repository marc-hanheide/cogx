package cdsr.data;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import cdsr.objects.AnnotatedCDSR;

/**
 * This file reads the results for lots of subjects.
 * 
 * @author Graham Horn
 * 
 */
public class SubjectResultsReader
{

  /**
   * 
   * @param file_listing_subject_data_files
   *          - one file (path) per line
   * @return a set of all results as AnnotatedCDSRs
   */
  public static Set<AnnotatedCDSR> processAllSubjectDataFiles(
      String file_listing_subject_data_files)
  {

    List<String> subject_data_files = new ArrayList<String>();
    try
    {
      BufferedReader reader = new BufferedReader(new FileReader(
          file_listing_subject_data_files));
      String subject_data_file_name;

      while ((subject_data_file_name = reader.readLine()) != null)
      {
        subject_data_files.add(subject_data_file_name);
      }
    }
    catch (IOException e)
    {
      e.printStackTrace();
    }

    return processAllSubjectDataFiles(subject_data_files);
  }

  public static Set<AnnotatedCDSR> processAllSubjectDataFiles(
      List<String> subject_data_files)
  {
    HashSet<AnnotatedCDSR> annotatedCdrs = new HashSet<AnnotatedCDSR>();
    Iterator<String> itr = subject_data_files.iterator();
    while (itr.hasNext())
    {
      String next_file = itr.next();
      try
      {
        annotatedCdrs.addAll(SubjectDataReader
            .processSubjectDataFile(next_file));
      }
      catch (IOException e)
      {
        e.printStackTrace();
        System.out.println("Error processing file " + next_file);
      }
    }
    return annotatedCdrs;
  }
}
