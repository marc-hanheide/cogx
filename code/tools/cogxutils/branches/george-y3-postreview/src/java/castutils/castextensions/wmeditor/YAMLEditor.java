package castutils.castextensions.wmeditor;

import castutils.castextensions.wmeditor.serializer.Serializer;
import castutils.castextensions.wmeditor.serializer.YAMLSerializer;

public class YAMLEditor extends WMEditorComponent {

	final Serializer serializer = new YAMLSerializer();
	private static final String TEMPLATEFILE = YAMLEditor.class.getSimpleName()
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
