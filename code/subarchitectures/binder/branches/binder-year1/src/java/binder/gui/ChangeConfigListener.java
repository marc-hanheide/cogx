package binder.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JPanel;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.components.BinderMonitor;

public class ChangeConfigListener implements ActionListener {
	
	WestArrowPanel.DIRECTION dir;
	
	BinderMonitor bm;
	
	WestArrowPanel panel;
	
	static boolean LOGGING = true;
	
	private static Logger logger = ComponentLogger.getLogger(ChangeConfigListener.class);

	
	public ChangeConfigListener(BinderMonitor bm, WestArrowPanel panel, WestArrowPanel.DIRECTION dir) {
		this.bm = bm;
		this.dir = dir;
		this.panel = panel;
	}

	public void actionPerformed(ActionEvent e) {

		if (dir.equals(WestArrowPanel.DIRECTION.UP)) {
			if (panel.getCurrentRank() > 1) {
				int newRank = panel.getCurrentRank() - 1;
				bm.showConfigurationOfRank(newRank);
				panel.setCurrentRank(newRank);
			}
			else {
				log("(BOOOOOONG) maximum rank reached!");
			}
		}

		else if (dir.equals(WestArrowPanel.DIRECTION.DOWN)) {
			if (panel.getCurrentRank() < bm.alternativeConfigs.alterconfigs.length) {
				int newRank = panel.getCurrentRank() + 1;
				bm.showConfigurationOfRank(newRank);
				panel.setCurrentRank(newRank);
			}
			else {
				log("(BOOOOOONG) minimum rank reached!");
			}
		}
	}

	
	public void log(String s) {
		if (LOGGING) {
			logger.debug("[ChangeConfigListener] " + s);
		}
			
	}
}
