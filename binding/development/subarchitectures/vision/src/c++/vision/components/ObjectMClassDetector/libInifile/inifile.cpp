//
// IniFile Class Implementation
//
// The purpose of this class is to provide a simple, full featured means to
// store persistent data to a text file.  It uses a simple key/value paradigm
// to achieve this.  The class can read/write to standard Windows .ini files,
// and yet does not rely on any windows specific calls.  It should work as
// well in a linux environment (with some minor adjustments) as it does in
// a Windows one.
//
// Written July, 2002 by Gary McNickle <gary#sunstorm.net>




// If you use this class in your application, credit would be appreciated.
//

//
// IniFile
// The purpose of this class is to provide the means to easily store key/value
// pairs in a config file, seperated by independant sections. Sections may not
// have duplicate keys, although two or more sections can have the same key.
// Simple support for comments is included. Each key, and each section may have
// it's own multiline comment.
//
// An example might look like this;
//
// [UserSettings]
// Name=Joe User
// Date of Birth=12/25/01
//
// ;
// ; Settings unique to this server
// ;
// [ServerSettings]
// Port=1200
// IP_Address=127.0.0.1
// MachineName=ADMIN
//

#include <vector>
#include <string>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <float.h>

#include <ios>
using namespace std;

#ifdef WIN32
#include <windows.h>
#endif

#include <libInifile/inifile.h>
using namespace inifile;

// Compatibility Defines ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
  #define snprintf  _snprintf
  #define vsnprintf _vsnprintf
#endif

// CommentIndicators
// This constant contains the characters that we check for to determine if a
// line is a comment or not. Note that the first character in this constant is
// the one used when writing comments to disk (if the comment does not allready
// contain an indicator)
static t_Str CommentIndicators = t_Str(";#");

// EqualIndicators
// This constant contains the characters that we check against to determine if
// a line contains an assignment ( key = value )
// Note that changing these from their defaults ("=:") WILL affect the
// ability of IniFile to read/write to .ini files.  Also, note that the
// first character in this constant is the one that is used when writing the
// values to the file. (EqualIndicators[0])
static t_Str EqualIndicators   = t_Str("=:");

// WhiteSpace
// This constant contains the characters that the Trim() function removes from
// the head and tail of strings.
static t_Str WhiteSpace = t_Str(" \t\n\r");

/// General Purpose Utility Functions ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
static t_Str	GetNextWord(t_Str& CommandLine);
static int		CompareNoCase(const t_Str& str1, const t_Str& str2);
static void	Trim(t_Str& szStr);
static int		WriteLn(std::fstream& stream, const char* fmt, ...);

IniFile::IniFile ()
{
	Clear();
	m_Flags = (AUTOCREATE_SECTIONS | AUTOCREATE_KEYS);
	m_bShowErrors = false;	
	m_Sections.push_back( *(new t_Section) );	
	
}

IniFile::IniFile(bool showErrors) {	
	Clear();
	m_Flags = (AUTOCREATE_SECTIONS | AUTOCREATE_KEYS);
	m_bShowErrors = showErrors;
	m_Sections.push_back( *(new t_Section) );
}
	


// ~IniFile
// Saves the file if any values have changed since the last save.
IniFile::~IniFile()
{
	/* REM BY KAI
	if ( m_bDirty )
		Save();
	*/
}

// Clear
// Resets the member variables to their defaults
void IniFile::Clear()
{
	m_bDirty = false;
	m_szFileName = t_Str("");
	m_Sections.clear();
}

// SetFileName
// Set's the m_szFileName member variable. For use when creating the IniFile
// object by hand (-vs- loading it from a file
void IniFile::SetFileName(t_Str szFileName)
{
	if (m_szFileName.size() != 0 && CompareNoCase(szFileName, m_szFileName) != 0)
	{
		m_bDirty = true;
	}

	m_szFileName = szFileName;
}



// Load
// Attempts to load in the text file. If successful it will populate the
// Section list with the key/value pairs found in the file. Note that comments
// are saved so that they can be rewritten to the file later.
bool IniFile::Load(t_Str szFileName)
{
	// We dont want to create a new file here.  If it doesn't exist, just
	// return false and report the failure.
	
	//fstream File(szFileName.c_str(), fstream::in|fstream::nocreate);
	fstream File(szFileName.c_str(), fstream::in);

	if ( File.is_open() )
	{
		bool bDone = false;
		bool bAutoKey = (m_Flags & AUTOCREATE_KEYS) == AUTOCREATE_KEYS;
		bool bAutoSec = (m_Flags & AUTOCREATE_SECTIONS) == AUTOCREATE_SECTIONS;
		
		t_Str szLine;
		t_Str szComment;
		char buffer[MAX_BUFFER_LEN]; 
		t_Section* pSection = GetSection("");

		// These need to be set, we'll restore the original values later.
		m_Flags |= AUTOCREATE_KEYS;
		m_Flags |= AUTOCREATE_SECTIONS;

		while ( !bDone )
		{
			memset(buffer, 0, MAX_BUFFER_LEN);
			File.getline(buffer, MAX_BUFFER_LEN);

			szLine = buffer;
			Trim(szLine);

			bDone = ( File.eof() || File.bad() || File.fail() );

			if ( szLine.find_first_of(CommentIndicators) == 0 )
			{
				szComment += "\n";
				szComment += szLine;
			}
			else
			if ( szLine.find_first_of('[') == 0 ) // new section
			{
				szLine.erase( 0, 1 );
				szLine.erase( szLine.find_last_of(']'), 1 );

				CreateSection(szLine, szComment);
				pSection = GetSection(szLine);
				szComment = t_Str("");
			}
			else 
			if ( szLine.size() > 0 ) // we have a key, add this key/value pair
			{
				t_Str szKey = GetNextWord(szLine);
				t_Str szValue = szLine;

				if ( szKey.size() > 0 && szValue.size() > 0 )
				{
					SetValue(szKey, szValue, szComment, pSection->szName);
					szComment = t_Str("");
				}
			}
		}

		// Restore the original flag values.
		if ( !bAutoKey )
			m_Flags &= ~AUTOCREATE_KEYS;

		if ( !bAutoSec )
			m_Flags &= ~AUTOCREATE_SECTIONS;
	}
	else
	{
		return false;
	}

	File.close();

	return true;
}


// Save
// Attempts to save the Section list and keys to the file. Note that if Load
// was never called (the IniFile object was created manually), then you
// must set the m_szFileName variable before calling save.
bool IniFile::Save()
{
	if ( KeyCount() == 0 && SectionCount() == 0 )
	{
		// no point in saving
		return false;
	}

	if ( m_szFileName.size() == 0 )
	{
		return false;
	}

	std::fstream File(m_szFileName.c_str(), ios::out|ios::trunc);

	if ( File.is_open() )
	{
		SectionItor s_pos;
		KeyItor k_pos;
		t_Section Section;
		t_Key Key;

		for (s_pos = m_Sections.begin(); s_pos != m_Sections.end(); s_pos++)
		{
			Section = (*s_pos);
			bool bWroteComment = false;

			if ( Section.szComment.size() > 0 )
			{
				bWroteComment = true;
				WriteLn(File, "\n%s", CommentStr(Section.szComment).c_str());
			}

			if ( Section.szName.size() > 0 )
			{
				WriteLn(File, "%s[%s]", 
						bWroteComment ? "" : "\n", 
						Section.szName.c_str());
			}

			for (k_pos = Section.Keys.begin(); k_pos != Section.Keys.end(); k_pos++)
			{
				Key = (*k_pos);

				if ( Key.szKey.size() > 0 && Key.szValue.size() > 0 )
				{
					WriteLn(File, "%s%s%s%s%c%s", 
						Key.szComment.size() > 0 ? "\n" : "",
						CommentStr(Key.szComment).c_str(),
						Key.szComment.size() > 0 ? "\n" : "",
						Key.szKey.c_str(),
						EqualIndicators[0],
						Key.szValue.c_str());
				}
			}
		}
		
	}
	else
	{
		return false;
	}

	m_bDirty = false;
	
	File.flush();
	File.close();
	return true;
}

// SetKeyComment
// Set the comment of a given key. Returns true if the key is not found.
bool IniFile::SetKeyComment(t_Str szKey, t_Str szComment, t_Str szSection)
{
	KeyItor k_pos;
	t_Section* pSection;

	if ( (pSection = GetSection(szSection)) == NULL )
		return false;

	for (k_pos = pSection->Keys.begin(); k_pos != pSection->Keys.end(); k_pos++)
	{
		if ( CompareNoCase( (*k_pos).szKey, szKey ) == 0 )
		{
			(*k_pos).szComment = szComment;
			m_bDirty = true;
			return true;
		}
	}

	return false;

}

// SetSectionComment
// Set the comment for a given section. Returns false if the section
// was not found.
bool IniFile::SetSectionComment(t_Str szSection, t_Str szComment)
{
	SectionItor s_pos;

	for (s_pos = m_Sections.begin(); s_pos != m_Sections.end(); s_pos++)
	{
		if ( CompareNoCase( (*s_pos).szName, szSection ) == 0 ) 
		{
		    (*s_pos).szComment = szComment;
			m_bDirty = true;
			return true;
		}
	}

	return false;
}


// SetValue
// Given a key, a value and a section, this function will attempt to locate the
// Key within the given section, and if it finds it, change the keys value to
// the new value. If it does not locate the key, it will create a new key with
// the proper value and place it in the section requested.
bool IniFile::SetValue(t_Str szKey, t_Str szValue, t_Str szComment, t_Str szSection)
{
	t_Key* pKey = GetKey(szKey, szSection);
	t_Section* pSection = GetSection(szSection);

	if (pSection == NULL)
	{
		if ( !(m_Flags & AUTOCREATE_SECTIONS) || !CreateSection(szSection,""))
			return false;

		pSection = GetSection(szSection);
	}

	// Sanity check...
	if ( pSection == NULL )
		return false;

	// if the key does not exist in that section, and the value passed 
	// is not t_Str("") then add the new key.
	if ( pKey == NULL && szValue.size() > 0 && (m_Flags & AUTOCREATE_KEYS))
	{
		pKey = new t_Key;

		pKey->szKey = szKey;
		pKey->szValue = szValue;
		pKey->szComment = szComment;
		
		m_bDirty = true;
		
		pSection->Keys.push_back(*pKey);

		return true;
	}

	if ( pKey != NULL )
	{
		pKey->szValue = szValue;
		pKey->szComment = szComment;

		m_bDirty = true;
		
		return true;
	}

	return false;
}

// SetFloat
// Passes the given float to SetValue as a string
bool IniFile::SetFloat(t_Str szKey, float fValue, t_Str szComment, t_Str szSection)
{
	char szStr[64];

	snprintf(szStr, 64, "%f", fValue);

	return SetValue(szKey, szStr, szComment, szSection);
}


// SetDouble
bool IniFile::SetDouble(t_Str szKey, double fValue, t_Str szComment, t_Str szSection)
{
	char szStr[64];

	snprintf(szStr, 64, "%.10f", fValue);

	return SetValue(szKey, szStr, szComment, szSection);
}



// SetInt
// Passes the given int to SetValue as a string
bool IniFile::SetInt(t_Str szKey, int nValue, t_Str szComment, t_Str szSection)
{
	char szStr[64];

	snprintf(szStr, 64, "%d", nValue);

	return SetValue(szKey, szStr, szComment, szSection);

}

// SetBool
// Passes the given bool to SetValue as a string
bool IniFile::SetBool(t_Str szKey, bool bValue, t_Str szComment, t_Str szSection)
{
	t_Str szValue = bValue ?  "True" : "False";

	return SetValue(szKey, szValue, szComment, szSection);
}

// GetValue
// Returns the key value as a t_Str object. A return value of
// t_Str("") indicates that the key could not be found.
t_Str IniFile::GetValue(t_Str szKey, t_Str szSection, t_Str def)  const
{
	t_Key* pKey = GetKey(szKey, szSection);
	
	if (pKey == NULL && m_bShowErrors)
		printf("Using standard value %s for key %s in section [%s]\n", def.c_str(), szKey.c_str(), szSection.c_str());
	return (pKey == NULL) ? def : pKey->szValue;
}

// GetString
// Returns the key value as a t_Str object. A return value of
// t_Str("") indicates that the key could not be found.
t_Str IniFile::GetString(t_Str szKey, t_Str szSection, t_Str def) const
{
	t_Str t = GetValue(szKey, szSection, def);
	Trim(t);
	return t;
}

// GetFloat
// Returns the key value as a float type. Returns FLT_MIN if the key is
// not found.
float IniFile::GetFloat(t_Str szKey, t_Str szSection, float def) const
{
	char val[64];
	
	snprintf(val, 64, "%f", def);
	
	t_Str szValue = GetValue(szKey, szSection, t_Str(val));

	if ( szValue.size() == 0 )
		return def;
		//return FLT_MIN;
		

	return (float)atof( szValue.c_str() );
}

// GetDouble
double IniFile::GetDouble(t_Str szKey, t_Str szSection, double def) const
{
	
	char val[64];
	
	snprintf(val, 64, "%f", def);
	
	t_Str szValue = GetValue(szKey, szSection, t_Str(val));

	if ( szValue.size() == 0 )
	  return def;
	  //return 0;
	  //return FLT_MIN;

	return atof( szValue.c_str() );
}

// GetInt
// Returns the key value as an integer type. Returns INT_MIN if the key is
// not found.
int	IniFile::GetInt(t_Str szKey, t_Str szSection, int def) const
{
	char val[64];	
	snprintf(val, 64, "%d", def);
		
	t_Str szValue = GetValue(szKey, szSection, t_Str(val));
	
	if ( szValue.size() == 0 )
		return def;
		//return INT_MIN;

	return atoi( szValue.c_str() );
}

// GetBool
// Returns the key value as a bool type. Returns false if the key is
// not found.
bool IniFile::GetBool(t_Str szKey, t_Str szSection, bool def) const
{
	bool bValue = false;
	t_Str szValue = GetValue(szKey, szSection, def ? "true" : "false" );
	//printf("iniboolvalue: %s\n",szValue.c_str());
	if ( szValue.find("1") != std::string::npos 
		|| CompareNoCase(szValue, "true") == 0 
		|| CompareNoCase(szValue, "yes") == 0 )
	{
		bValue = true;
	}

	return bValue;
}

// DeleteSection
// Delete a specific section. Returns false if the section cannot be 
// found or true when sucessfully deleted.
bool IniFile::DeleteSection(t_Str szSection)
{
	SectionItor s_pos;

	for (s_pos = m_Sections.begin(); s_pos != m_Sections.end(); s_pos++)
	{
		if ( CompareNoCase( (*s_pos).szName, szSection ) == 0 ) 
		{
			m_Sections.erase(s_pos);
			return true;
		}
	}

	return false;
}

// DeleteKey
// Delete a specific key in a specific section. Returns false if the key
// cannot be found or true when sucessfully deleted.
bool IniFile::DeleteKey(t_Str szKey, t_Str szFromSection)
{
	KeyItor k_pos;
	t_Section* pSection;

	if ( (pSection = GetSection(szFromSection)) == NULL )
		return false;

	for (k_pos = pSection->Keys.begin(); k_pos != pSection->Keys.end(); k_pos++)
	{
		if ( CompareNoCase( (*k_pos).szKey, szKey ) == 0 )
		{
			pSection->Keys.erase(k_pos);
			return true;
		}
	}

	return false;
}

// CreateKey
// Given a key, a value and a section, this function will attempt to locate the
// Key within the given section, and if it finds it, change the keys value to
// the new value. If it does not locate the key, it will create a new key with
// the proper value and place it in the section requested.
bool IniFile::CreateKey(t_Str szKey, t_Str szValue, t_Str szComment, t_Str szSection)
{
	bool bAutoKey = (m_Flags & AUTOCREATE_KEYS) == AUTOCREATE_KEYS;
	bool bReturn  = false;

	m_Flags |= AUTOCREATE_KEYS;

	bReturn = SetValue(szKey, szValue, szComment, szSection);

	if ( !bAutoKey )
		m_Flags &= ~AUTOCREATE_KEYS;

	return bReturn;
}


// CreateSection
// Given a section name, this function first checks to see if the given section
// allready exists in the list or not, if not, it creates the new section and
// assigns it the comment given in szComment.  The function returns true if
// sucessfully created, or false otherwise. 
bool IniFile::CreateSection(t_Str szSection, t_Str szComment)
{
	t_Section* pSection = GetSection(szSection);

	if ( pSection )
	{
		return false;
	}

	pSection = new t_Section;

	pSection->szName = szSection;
	pSection->szComment = szComment;
	m_Sections.push_back(*pSection);
	m_bDirty = true;

	return true;
}

// CreateSection
// Given a section name, this function first checks to see if the given section
// allready exists in the list or not, if not, it creates the new section and
// assigns it the comment given in szComment.  The function returns true if
// sucessfully created, or false otherwise. This version accpets a KeyList 
// and sets up the newly created Section with the keys in the list.
bool IniFile::CreateSection(t_Str szSection, t_Str szComment, KeyList Keys)
{
	if ( !CreateSection(szSection, szComment) )
		return false;

	t_Section* pSection = GetSection(szSection);

	if ( !pSection )
		return false;

	KeyItor k_pos;

	pSection->szName = szSection;
	for (k_pos = Keys.begin(); k_pos != Keys.end(); k_pos++)
	{
		t_Key* pKey = new t_Key;
		pKey->szComment = (*k_pos).szComment;
		pKey->szKey = (*k_pos).szKey;
		pKey->szValue = (*k_pos).szValue;

		pSection->Keys.push_back(*pKey);
	}

	m_Sections.push_back(*pSection);
	m_bDirty = true;

	return true;
}

// SectionCount
// Simply returns the number of sections in the list.
int IniFile::SectionCount() 
{ 
	return m_Sections.size(); 
}

// KeyCount
// Returns the total number of keys contained within all the sections.
int IniFile::KeyCount()
{
	int nCounter = 0;
	SectionItor s_pos;

	for (s_pos = m_Sections.begin(); s_pos != m_Sections.end(); s_pos++)
		nCounter += (*s_pos).Keys.size();

	return nCounter;
}


// Protected Member Functions ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// GetKey
// Given a key and section name, looks up the key and if found, returns a
// pointer to that key, otherwise returns NULL.
t_Key*	IniFile::GetKey(t_Str szKey, t_Str szSection) const

{	
	t_Section* pSection;

	// Since our default section has a name value of t_Str("") this should
	// always return a valid section, wether or not it has any keys in it is
	// another matter.
	if ( (pSection = GetSection(szSection)) == NULL )
		return NULL;

	for (KeyList::const_iterator k_pos = pSection->Keys.begin(); k_pos != pSection->Keys.end(); k_pos++)
	{
		if ( CompareNoCase( (*k_pos).szKey, szKey ) == 0 )
			return (t_Key*)&(*k_pos);
	}

	return NULL;
}

// GetSection
// Given a section name, locates that section in the list and returns a pointer
// to it. If the section was not found, returns NULL
t_Section* IniFile::GetSection(t_Str szSection) const
{
	for (SectionList::const_iterator s_pos = m_Sections.begin(); s_pos != m_Sections.end(); s_pos++)
	{
		if ( CompareNoCase( (*s_pos).szName, szSection ) == 0 ) 
			return (t_Section*)&(*s_pos);
	}

	return NULL;
}


t_Str IniFile::CommentStr(t_Str szComment)
{
	t_Str szNewStr = t_Str("");

	Trim(szComment);

        if ( szComment.size() == 0 )
          return szComment;

	
	if ( szComment.find_first_of(CommentIndicators) != 0 )
	{
		szNewStr = CommentIndicators[0];
		szNewStr += " ";
	}

	szNewStr += szComment;

	return szNewStr;
}



// Utility Functions ////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// GetNextWord
// Given a key +delimiter+ value string, pulls the key name from the string,
// deletes the delimiter and alters the original string to contain the
// remainder.  Returns the key
static t_Str GetNextWord(t_Str& CommandLine)
{
	string::size_type nPos = CommandLine.find_first_of(EqualIndicators);
	t_Str sWord = t_Str("");

	if ( nPos != string::npos )
	{
		sWord = CommandLine.substr(0, nPos);
		CommandLine.erase(0, nPos+1);
	}
	else
	{
		sWord = CommandLine;
		CommandLine = t_Str("");
	}

	Trim(sWord);
	return sWord;
}


// CompareNoCase
// it's amazing what features std::string lacks.  This function simply
// does a lowercase compare against the two strings, returning 0 if they
// match.
static int CompareNoCase(const t_Str& str1, const t_Str& str2)
{
#ifdef WIN32
	return stricmp(str1.c_str(), str2.c_str());	
#else
	return strcasecmp(str1.c_str(), str2.c_str());
#endif
}

// Trim
// Trims whitespace from both sides of a string.
static void Trim(t_Str& szStr)
{
	t_Str szTrimChars = WhiteSpace;
	
	szTrimChars += EqualIndicators;
	string::size_type nPos, rPos;

	// trim left
	nPos = szStr.find_first_not_of(szTrimChars);

	if ( nPos > 0 )
		szStr.erase(0, nPos);

	// trim right and return
	nPos = szStr.find_last_not_of(szTrimChars);
	rPos = szStr.find_last_of(szTrimChars);

	if ( rPos > nPos && rPos != string::npos)
		szStr.erase(rPos, szStr.size()-rPos);
}

// WriteLn
// Writes the formatted output to the file stream, returning the number of
// bytes written.
static int WriteLn(std::fstream& stream, const char* fmt, ...)
{
	char buf[MAX_BUFFER_LEN];
	int nLength;
	t_Str szMsg;

	memset(buf, 0, MAX_BUFFER_LEN);
	va_list args;

	va_start (args, fmt);
	  nLength = vsnprintf(buf, MAX_BUFFER_LEN, fmt, args);
	va_end (args);


	if ( buf[nLength] != '\n' && buf[nLength] != '\r' )
		buf[nLength++] = '\n';


	stream.write(buf, nLength);

	return nLength;
}


