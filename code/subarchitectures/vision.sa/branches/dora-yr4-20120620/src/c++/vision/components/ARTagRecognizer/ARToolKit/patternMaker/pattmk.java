// Copyright (c) 2002 The University of Utah
// All Rights Reserved. 
// 
// This is an experimental, research release of a component of the
// Alpha_1 Geometric Modelling System, and as such is subject to a
// license agreement with Engineering Geometry Systems, Inc. and may only
// be used under the terms of that agreement.  Any other use is
// prohibited.
// 
// Having received prior authorization from EGS, the University of Utah grants
// permission to use, copy, or modify this software and its
// documentation for educational, research and non-profit purposes,
// without fee, and without a written agreement.  It is explicitly prohibited to
// redistribute this software or any derivative. 
// 
//       IN NO EVENT SHALL THE UNIVERSITY OF UTAH OR ENGINEERING GEOMETRY
//       SYSTEMS, INC.  BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
//       SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
//       PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
//       DOCUMENTATION, EVEN IF THE UNIVERSITY OF UTAH HAVE BEEN ADVISED
//       OF THE POSSIBILITY OF SUCH DAMAGES.
// 
//       THE UNIVERSITY OF UTAH AND ENGINEERING GEOMETRY SYSTEMS, INC.
//       SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
//       TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
//       PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN
//       "AS IS" BASIS, AND THE UNIVERSITY OF UTAH AND ENGINEERING
//       GEOMETRY SYSTEMS, INC. HAVE NO OBLIGATIONS TO PROVIDE
//       MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
// 
// The above permission is granted provided that this whole notice
// appears in all copies.
// 
// Contacts are: 
// 
//       EMail:  {dejohnso, cohen} @ cs.utah.edu 

import java.io.*;
import java.lang.*;

public class pattmk {

    private static int validSize = 4;
    
    public static void genText (String imageName, String line, int size, int header) throws IOException {
	
	int dup = 0;
	if (size == 4) {
	    dup = 4; // number of duplications for a 4x4
	}
	else if (size == 3) {
	    dup = 5; // number of duplications for a 3x3
	}
	File obj = new File(imageName);
	String tempPattS = line;  
	PrintWriter pw = new PrintWriter(new FileWriter(obj));
	if (header == 1) {
	    pw.println("P2");
	    pw.println("# Created by Bryan Witkowski 6/14/02");
	    pw.println(16 + " " + 192);
	    pw.println("255");
	}
	char tempC;
	for (int u = 0; u < 4; u++) {
	    for (int w = 0; w < 3; w++) { // each pattern is duped 3 times 
		for (int x = 0; x < size; x++) {    // for each line of pattern
		    for (int z = 0; z < dup; z++) {   // do each line multiple times			
			for (int n = 0; n < size; n++) {
			    tempC = tempPattS.charAt(n * 2 + size * x * 2);
			    if (tempC == '0') {
				for (int y = 0; y < dup; y++)
				    pw.print("255 ");
			    }
			    if (tempC == '1') {
				for (int y = 0; y < dup; y++) 
				    pw.print("0     ");
			    }
			}	
			
			pw.println();
		    }
		}
	    }
	    pw.println(); // add a blank line between rotations
	    
	    tempPattS = rotate(tempPattS);  // rotate 3 times right
	}
	
	
	pw.close();
    }
    
    // written by chris
    public static int RIDX(int i, int j, int n) { 
	int rtn = (i)*(n)+(j);
	return rtn;
    }
    
    public static int [] str2array(String pattS) {
	int dim  = pattS.length() / 2;
	int [] rtn = new int[dim];
	for (int i = 0; i < dim; i++) {
	    rtn[i] = Integer.parseInt(pattS.charAt(i*2) + "");	
	}
	return rtn;
    }
    
    public static String array2str(int [] inarray) {
	int dim  = inarray.length;
	String rtn = "";
	for (int i = 0; i < dim; i++) {
	    rtn = rtn + inarray[i] + " ";
	}
	return rtn;
    }
    
    public static String rotate(String inStr){
	int [] src = str2array(inStr);
	int dim = (int)Math.sqrt(src.length);
	int [] rtn = new int[dim * dim];
	int i, j;
	for (i = 0; i < dim; ++i)
	    for (j = 0; j < dim; ++j)
		rtn[RIDX(dim-1-j, i, dim)] = src[RIDX(i, j, dim)];
	return (array2str(rtn));
    }
    
    public static void main (String [] args) throws IOException{
	try
	{
	    
	    if (args.length <= 2)
		throw new Exception("Usage: java pattmk <#to create> <dim size> <ppm image 1|0>");
	    
	    String filename = "1";
	    int size = Integer.parseInt(args[1]);
	    int head = Integer.parseInt(args[2]);
	    File inf;
	    BufferedReader br;
	    String tempPattS;

	    if (size != validSize)
		throw new Exception("Only " + validSize + "x" + validSize + " patterns are supported right now.");

	    inf = new File(size + "x" + size);
	    br = new BufferedReader(new FileReader(inf));
	    tempPattS = br.readLine();
	    tempPattS = "";
	    for (int i = 0; i < Integer.parseInt(args[0]); i++)
	    {
		for (int m = 0; m < size; m++)
		{
		    tempPattS += br.readLine();
		}
		if (head == 1)
		    genText(size + "x" + size + "_" + filename + ".ppm", tempPattS, size, head);
		else if (head == 0)
		    genText(size + "x" + size + "_" + filename + ".patt", tempPattS, size, head);
		filename = new String(Integer.parseInt(filename) + 1 + "");
		tempPattS = br.readLine();
		
	    }
	}
	catch (Exception e)
	{
	    System.out.println(e.getMessage());
	}	
    }
}
