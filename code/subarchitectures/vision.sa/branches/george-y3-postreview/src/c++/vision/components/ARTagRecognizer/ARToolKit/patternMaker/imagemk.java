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

public class imagemk {

    public static void genText (String imageName, String line, int size, int quality) throws IOException {
	
	int dup = 0;
	int dim = 24 * quality;
	if (size == 4) {
	    dup = dim / 8; // number of duplications for a 4x4
	}
	else if (size == 3) {
	    dup = dim / 6; // number of duplications for a 3x3
	}
	File obj = new File(imageName);
	String tempPattS = line;  
	PrintWriter pw = new PrintWriter(new FileWriter(obj));
	pw.println("P2");
	pw.println("# Created by Bryan Witkowski 5/30/02");
	pw.println(dim + " " + dim);
	pw.println("255");
	
	for (int j = 0; j < dim / 4; j++) {
	    for (int i = 0; i < dim; i++) 
		pw.print("0     ");
	    pw.println();
	}
	
	char tempC;
	for (int x = 0; x < size; x++) {    // for each line of pattern
	    for (int z = 0; z < dup; z++) {   // do each line multiple times
		for (int j = 0; j < dim / 4; j++) 
		    pw.print("0     ");
		
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
		
		for (int j = 0; j < dim / 4; j++) 
				pw.print("0     ");
		pw.println();
	    }
	}
	
	for (int j = 0; j < dim / 4; j++) { 
	    for (int i = 0; i < dim; i++) 
		pw.print("0     ");
	    pw.println();
	}
	
	pw.close();
    }
    
    public static void main (String [] args) throws IOException{
	if (args.length > 2) {
	    String filename = "1";
	    int size = Integer.parseInt(args[1]);
	    int qual = Integer.parseInt(args[2]);
	    File inf = new File(size + "x" + size);
	    BufferedReader br = new BufferedReader(new FileReader(inf));
	    String tempPattS = br.readLine();
	    tempPattS = "";

	    for (int i = 0; i < Integer.parseInt(args[0]); i++) {
		for (int m = 0; m < size; m++) {
		    tempPattS += br.readLine();
		}
		genText(size + "x" + size + "_" + qual*24 + "_" + filename + ".ppm", tempPattS, size, qual);
		filename = new String(Integer.parseInt(filename) + 1 + "");
		tempPattS = br.readLine();
	    }
	}
	else 
	    System.out.println("Usage: java imagemk <#to create> <dim size> <img size 1:16>");
    }
}
