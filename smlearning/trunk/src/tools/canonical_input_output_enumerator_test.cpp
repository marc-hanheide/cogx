#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string target_dir;
	string cfg;
	int modulo = 0;
	if (argc >= 5) {
		target_dir = string (argv[4]);
	}
	if (argc >= 4) {
		modulo = atoi(argv[3]);
		cout << modulo << endl;
	}
	if (argc >= 3) {
		cfg.assign (argv[2]);
		seqFile = string (argv[1]);
	}
	else {
		cerr << argv[0] << " sequence_file (without extension) config_xml_file [modulo (default: 1)] [target_dir (default:current file dir.)]" << endl;
		return 1;
	}

	if (modulo <= 0) {
		cout << "modulo assumed to be 1" << endl;
		modulo = 1;
	}
	
	DataSetStruct savedData;

	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	XMLParser::Desc parserDesc;
	XMLParser::Ptr pParser = parserDesc.create ();
	try {
		FileReadStream fs (cfg.c_str());
		pParser->load (fs);
	}
	catch (const Message& msg) {
		std::cerr << msg << std::endl;
		return 1;
	}
	XMLContext *pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
	if (pXMLContext == NULL)
		throw MsgSystem(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());
	Scenario::Desc desc;
	XMLData(desc, pXMLContext);

	
	CanonicalData::DataSetStruct newData = canonical_input_output_enumerator_with_time (savedData, desc, modulo);

	if (argc == 4)
		// write_canonical_dataset_cryssmex_fmt (target_dir + "/" + seqBaseFileName, newData);
		write_canonical_dataset_cryssmex_fmt_regression (target_dir + "/" + seqBaseFileName, newData);
	else if (argc == 3 || argc == 2)
		// write_canonical_dataset_cryssmex_fmt (seqFile, newData);
		write_canonical_dataset_cryssmex_fmt_regression (seqFile, newData);
	return 0;
}
