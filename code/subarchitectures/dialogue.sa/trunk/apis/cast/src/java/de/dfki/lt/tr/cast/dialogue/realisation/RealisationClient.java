package de.dfki.lt.tr.cast.dialogue.realisation;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;

public class RealisationClient implements LFRealiser {

	private Socket         m_socket;
	private PrintWriter    m_out;
	private BufferedReader m_in;

	private String  m_hostname = "localhost";
	private Integer m_port     =  4321;
	
	private static final int TIMEOUT = 1000;
	
	public RealisationClient(String hostname, Integer port) throws UnknownHostException, IOException {
		if (!hostname.equals("")) m_hostname = hostname;
		if (port!=null) m_port = port;
		initSocket();
	}
	
	private void initSocket() throws UnknownHostException, IOException {
        m_socket = new Socket(m_hostname, m_port);
        m_out = new PrintWriter(m_socket.getOutputStream(), true);
        m_in = new BufferedReader(new InputStreamReader(m_socket.getInputStream()));						
	}
	
	/**
	 * This method tries to parse the lfString into a BasicLogicalForm
	 * and then invokes realiseLF. 
	 * 
	 * @param lfString
	 * @throws BuildException, ParseException if the provided lfString is not valid
	 * @return a natural language surface realisation of the lfString
	 * or the empty String in case no realisation is found
	 */
	@Override
	public String realiseString(String lfString) throws BuildException, ParseException {
		while (m_out.checkError()) {
			System.err.println("socket connection broken. trying to reconnect in " + TIMEOUT);
			try {
				synchronized(this) {
					this.wait(TIMEOUT);
				}
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
			try {
				initSocket();
			} catch (UnknownHostException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		m_out.println(lfString);
		//Receive text from server
		try{
			String serverReply = m_in.readLine();
			if (serverReply==null) return realiseString(lfString);

			if (serverReply.startsWith("ERROR")) {
				if (serverReply.contains("ParseException")) {
					throw new ParseException(lfString,serverReply,-42);
				} else {
					throw new BuildException(new Throwable(serverReply));
				}
			} else {
				return serverReply;
			}
		} catch (IOException e){
			e.printStackTrace();
			return "";
        }	
    }

	/**
	 * This method realises a given BasicLogicalForm into a surface String 
	 * as defined by the grammar realiser. 
	 * 
	 * @param blf
	 * @return a natural language surface realisation of the blf 
	 * or the empty String in case no realisation is found
	 */
	@Override
	public String realiseLF(BasicLogicalForm blf) {
		try {
			String realisation = realiseString(blf.toString());
			return realisation;
		} catch (BuildException e) {
			e.printStackTrace();
			return "";
		} catch (ParseException e) {
			e.printStackTrace();
			return "";
		}
		
	}

}