package cdsr.data;

import java.awt.geom.Line2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import math.geom2d.Point2D;
import math.geom2d.polygon.Polygon2D;
import math.geom2d.polygon.Polygon2DUtils;
import math.geom2d.polygon.SimplePolygon2D;
import cdsr.gui.ImageGeneratorForPaper;
import cdsr.objects.CDSR;

public class RelativeAreaOverlapGenerator {

	// map room (filename) and CDSR type to data ((Map user to CDSR))
	private Map<String, Map<String, Map<String, CDSR>>> cdsrTypeTofilenameToUserToCdsr;
	
	public RelativeAreaOverlapGenerator()
	{
		cdsrTypeTofilenameToUserToCdsr = new HashMap<String, Map<String, Map<String, CDSR>>>();
	}
	
	public void processUserDataFile(String user_data_file) throws IOException
	{
		BufferedReader reader = new BufferedReader(new FileReader(user_data_file));
		int firstHyphen = user_data_file.indexOf('-');
		int firstPeriod = user_data_file.indexOf('.');
		String user = user_data_file.substring(firstHyphen+1, firstPeriod);
		System.out.println("user name = " + user);
		String next_line;
		while ((next_line = reader.readLine()) != null)
		{
			processLine(user, next_line);
		}
	}
	
	public static CDSR createCDSRFromDataString(String _data) {
		// WARNING this will fail to process negative numbers sensibly
		String[] split = _data.split("[ ()]+");
		ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>();

		int lastSuccess = 0;
		for (int i = 0; i < split.length - 3; i += 4) {
			try {

				lines.add(new Line2D.Double(Double.parseDouble(split[i]),
						Double.parseDouble(split[i + 1]), Double
								.parseDouble(split[i + 2]), Double
								.parseDouble(split[i + 3])));
				lastSuccess = (i + 3);
			} catch (NumberFormatException e) {
				System.out.println("Number format exception");
				// HACK this not really a good way of solving this problem
			}
		}
		
		String type = split[split.length - 1];
		
		return new CDSR(lines, type);
	}
	
	public void processLine(String user, String line)
	{
		int firstSpace = line.indexOf(' ');
		String cdsrDataFilename = line.substring(0, firstSpace);
		String groundTruthData = line.substring(firstSpace + 1);
		CDSR baseRegion = createCDSRFromDataString(groundTruthData);
		
		Map<String, Map<String, CDSR>> filenameToUserToCdsr = 
				cdsrTypeTofilenameToUserToCdsr.get(baseRegion.getType());
		
		if (filenameToUserToCdsr == null)
		{
			filenameToUserToCdsr = new HashMap<String, Map<String, CDSR>>();
			cdsrTypeTofilenameToUserToCdsr.put(baseRegion.getType(), filenameToUserToCdsr);
		}
		
		Map<String, CDSR> userToCdsr = filenameToUserToCdsr.get(cdsrDataFilename);
		if (userToCdsr == null)
		{
			userToCdsr = new HashMap<String, CDSR>();
			filenameToUserToCdsr.put(cdsrDataFilename, userToCdsr);
		}
		userToCdsr.put(user, baseRegion);
		
	}

	public void generateStatistics()
	{
		Iterator<String> region_type_itr = cdsrTypeTofilenameToUserToCdsr.keySet().iterator();
		int region_count = 0;
		double overall_average = 0;
		while (region_type_itr.hasNext())
		{
			String region_type = region_type_itr.next();
			region_count++;
			System.out.println("Region type = " + region_type);
			
			Map<String, Map<String, CDSR>> filenameToUserToCdsr = 
					cdsrTypeTofilenameToUserToCdsr.get(region_type);
			Iterator<String> room_itr = filenameToUserToCdsr.keySet().iterator();
			
			double region_type_overlap_sum = 0;
			int count = 0;
			
			while (room_itr.hasNext())
			{
				String room_name = room_itr.next();
//				System.out.println(room_name);
				
				Map<String, CDSR> userToCdsr = filenameToUserToCdsr.get(room_name);
				
				
				try
				{
					double relative_overlap = computeRelativeAreaOverlapForRegion(userToCdsr);
					region_type_overlap_sum += relative_overlap;
					System.out.println(room_name + ", " + relative_overlap);
					count++;
				}
				catch (NullPointerException e)
				{
					System.out.println("Skipping region " + region_type + " for " + room_name);
				}
			}
			
			double region_average = region_type_overlap_sum / count;
			System.out.println("Region average " + region_type + " " + region_average); 
			overall_average += region_average;
		}
		
		System.out.println("Overall avg " + (overall_average / region_count));
	}
	
	public double computeRelativeAreaOverlapForRegion(Map<String, CDSR> user_to_cdsr) throws NullPointerException
	{
		
		Iterator<String> itr = user_to_cdsr.keySet().iterator();
		List<Polygon2D> polygons = new ArrayList<Polygon2D>();
		while(itr.hasNext())
		{
			String user = itr.next();
//			System.out.println(user);
			CDSR cdsr = user_to_cdsr.get(user);
			SimplePolygon2D polygon = new SimplePolygon2D();

			List<Line2D.Double> lines = cdsr.getLines();
			polygon.addVertex(new Point2D(lines.get(0).getX1(), lines.get(0).getY1()));			
			for (int i = 0; i < lines.size(); i++) {
				polygon.addVertex(new Point2D(lines.get(i).getX2(), lines.get(i).getY2()));
			}
			polygons.add(polygon);
		}
		
		// TODO draw all of these polygons on the appropriate image
		
		Polygon2D intersection = computeIntersection(polygons);
		Polygon2D union = computeUnion(polygons);
		
//		System.out.println(intersection.getVertices());
//		System.out.println(union.getVertices());
		
		return intersection.getArea() / union.getArea();
		
	}
	
	
	
	public static double computeRelativeAreaOverlap(Polygon2D polygon1, Polygon2D polygon2)
	{
		Polygon2D intersection_polygon = Polygon2DUtils.intersection(polygon1, polygon2);		
		Polygon2D union_polygon = Polygon2DUtils.union(polygon1, polygon2);
		
		double area_of_intersection = intersection_polygon.getArea();
		double area_of_union = union_polygon.getArea();
		
	System.out.println("intersection " + area_of_intersection + " union " + area_of_union + " relative area overlap " + area_of_intersection/area_of_union);
		
		return area_of_intersection / area_of_union;
	}
	
	
	public static Polygon2D computeIntersection(List<Polygon2D> polygons)
	{
		Polygon2D result;
		if (polygons.size() == 0)
		{
			result = null;
		}
		else if (polygons.size() == 1)
		{
			result = polygons.get(0);
		}
		else
		{
			Polygon2D temp = null;
			Polygon2D next = null;
			try
			{
				Iterator<Polygon2D> itr = polygons.iterator();
				temp = itr.next();
				while (itr.hasNext())
				{
					next = itr.next();
					temp = Polygon2DUtils.intersection(temp, next);
				}
				result = temp;
			}
			catch (IllegalStateException e)
			{
//				e.printStackTrace();
//				System.out.println(temp.getVertices());
//				System.out.println(next.getVertices());
				System.out.println("bad data here");
				// TODO look at all of the input polygons - draw
				result = null;
			}
		}
		
		return result;
	}
	
	public static Polygon2D computeUnion(List<Polygon2D> polygons)
	{
		Polygon2D result;
		if (polygons.size() == 0)
		{
			result = null;
		}
		else if (polygons.size() == 1)
		{
			result = polygons.get(0);
		}
		else
		{
			Iterator<Polygon2D> itr = polygons.iterator();
			Polygon2D temp = itr.next();
			while (itr.hasNext())
			{
				temp = Polygon2DUtils.union(temp, itr.next());
			}
			result = temp;
		}
		
		return result;
	}
	
	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
			
//		SimplePolygon2D polygon1 = new SimplePolygon2D(new double[]{0, 6, 6, 0}, new double[]{0, 0, 10, 10});
//		SimplePolygon2D polygon2 = new SimplePolygon2D(new double[]{5, 10, 10, 5}, new double[]{0, 0, 10, 10});
//		SimplePolygon2D polygon3 = new SimplePolygon2D(new double[]{4, 7, 7, 4}, new double[]{5, 5, 15, 15});
//		
//		computeRelativeAreaOverlap(polygon1, polygon2);
//		computeRelativeAreaOverlap(polygon1, polygon3);
//		computeRelativeAreaOverlap(polygon3, polygon2);
//		
//		List<Polygon2D> polygons = new ArrayList<Polygon2D>();
//		polygons.add(polygon1);
//		polygons.add(polygon2);
//		polygons.add(polygon3);
//		
//		Polygon2D intersection = computeIntersection(polygons);
//		Polygon2D union = computeUnion(polygons);
//		
//		System.out.println(intersection.getVertices());
//		System.out.println(union.getVertices());
		
		if (args.length != 1) {
			System.out
					.println("Only one argument allowed/required, path to data file.");
			return;
		}
		
		RelativeAreaOverlapGenerator generator = new RelativeAreaOverlapGenerator();
		
		File specFile = new File(args[0]);
		BufferedReader reader = new BufferedReader(new FileReader(specFile));
		String user_file_name;
		while ((user_file_name = reader.readLine()) != null)
		{
			generator.processUserDataFile(user_file_name);
		}

		System.out.println(generator.cdsrTypeTofilenameToUserToCdsr);
		
		generator.generateStatistics();
		
	}

}
