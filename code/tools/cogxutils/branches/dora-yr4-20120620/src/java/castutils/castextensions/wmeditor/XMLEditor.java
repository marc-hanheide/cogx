package castutils.castextensions.wmeditor;

import castutils.castextensions.wmeditor.serializer.Serializer;
import castutils.castextensions.wmeditor.serializer.XMLSerializer;

public class XMLEditor extends WMEditorComponent {

	final Serializer serializer = new XMLSerializer();
	private static final String TEMPLATEFILE = XMLEditor.class.getSimpleName()
			+ ".tmpl";

	@Override
	protected Serializer getSerializer() {
		return serializer;
	}

	@Override
	protected String getTemplateFile() {
		return TEMPLATEFILE;
	}

}
