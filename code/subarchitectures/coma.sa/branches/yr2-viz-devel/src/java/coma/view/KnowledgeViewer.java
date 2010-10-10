package coma.view;

import java.io.IOException;

import coma.visulization.jung.ComaFrame;

public class KnowledgeViewer {
	private ComaFrame cf;
	public KnowledgeViewer(CrowlWrapperModel cm, String configFile) throws IOException
	{
		cf = new ComaFrame(cm, configFile);
	}
	
	
	public void show()
	{
		Thread t= new Thread();
		t.start();
		cf.waitFrame();
	}
}
