package de.dfki.lt.tr.dialogue.cplan.gui;

import de.dfki.lt.loot.gui.layouts.AbstractLayout;
import de.dfki.lt.loot.gui.layouts.BasicAtomLayout;
import de.dfki.lt.loot.gui.layouts.CompactConsLayout;
import de.dfki.lt.loot.gui.layouts.SimpleTreeLayout;

public class LFLayout extends AbstractLayout {
  public LFLayout() {
    LFMapFacetLayout.init();
    addLayout(new LFMapFacetLayout());
    addLayout(new CompactConsLayout());
    addLayout(new SimpleTreeLayout());
    addLayout(new BasicAtomLayout());
  }
}
