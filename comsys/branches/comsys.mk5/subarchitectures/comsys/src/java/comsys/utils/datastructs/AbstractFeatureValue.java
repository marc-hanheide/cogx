package comsys.utils.datastructs;

import java.util.Enumeration;
import java.util.Vector;

public interface AbstractFeatureValue {
		
	public String getFeat() ;
	
	public Object getValue() ;
	
	public boolean equals(AbstractFeatureValue fv) ;

}
