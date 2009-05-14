#ifndef __INIFILE_H__
#define __INIFILE_H__

/*
 *      inifile.h
 *
 *  based on CDataFile by Gary McNickle <gary#sunstorm.net>
 */


#include <vector>
#include <fstream>
#include <string>

namespace inifile
{

// AUTOCREATE_SECTIONS
// When set, this define will cause SetValue() to create a new section, if
// the requested section does not allready exist.
#define AUTOCREATE_SECTIONS     (1L<<1)

// AUOTCREATE_KEYS
// When set, this define causes SetValue() to create a new key, if the
// requested key does not allready exist.
#define AUTOCREATE_KEYS         (1L<<2)

// MAX_BUFFER_LEN
// Used simply as a max size of some internal buffers. Determines the maximum
// length of a line that will be read from or written to the file or the
// report output.
#define MAX_BUFFER_LEN				512


// eDebugLevel
// Used by our Report function to classify levels of reporting and severity
// of report.
enum e_DebugLevel
{
	// detailed programmatic informational messages used as an aid in
	// troubleshooting problems by programmers
	E_DEBUG = 0,
	// brief informative messages to use as an aid in troubleshooting
	// problems by production support and programmers
	E_INFO,
	// messages intended to notify help desk, production support and
	// programmers of possible issues with respect to the running application
	E_WARN,
	// messages that detail a programmatic error, these are typically
	// messages intended for help desk, production support, programmers and
	// occasionally users
	E_ERROR,
	// severe messages that are programmatic violations that will usually
	// result in application failure. These messages are intended for help
	// desk, production support, programmers and possibly users
	E_FATAL,
	// notice that all processing should be stopped immediately after the
	// log is written.
	E_CRITICAL
};


typedef std::string t_Str;



// st_key
// This structure stores the definition of a key. A key is a named identifier
// that is associated with a value. It may or may not have a comment.  All comments
// must PRECEDE the key on the line in the config file.
typedef struct st_key
{
	t_Str		szKey;
	t_Str		szValue;
	t_Str		szComment;

	st_key()
	{
		szKey = t_Str("");
		szValue = t_Str("");
		szComment = t_Str("");
	}

} t_Key;

typedef std::vector<t_Key> KeyList;
typedef KeyList::iterator KeyItor;

// st_section
// This structure stores the definition of a section. A section contains any number
// of keys (see st_keys), and may or may not have a comment. Like keys, all
// comments must precede the section.
typedef struct st_section
{
	t_Str		szName;
	t_Str		szComment;
	KeyList		Keys;

	st_section()
	{
		szName = t_Str("");
		szComment = t_Str("");
		Keys.clear();
	}

} t_Section;

typedef std::vector<t_Section> SectionList;
typedef SectionList::iterator SectionItor;





/// Class Definitions ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


// CDataFile
class IniFile
{
// Methods
public:
				// Constructors & Destructors
				/////////////////////////////////////////////////////////////////
				IniFile();
				IniFile(bool showErrors);
	virtual		~IniFile();

				// File handling methods
				/////////////////////////////////////////////////////////////////
	bool		Load(t_Str szFileName);
	bool		Save();

				// Data handling methods
				/////////////////////////////////////////////////////////////////

				// GetValue: Our default access method. Returns the raw t_Str value
				// Note that this returns keys specific to the given section only.
	t_Str		GetValue(t_Str szKey, t_Str szSection, t_Str def = t_Str("")) const;
				// GetString: Returns the value as a t_Str
	t_Str		GetString(t_Str szKey, t_Str szSection, t_Str def = t_Str("")) const;
				// GetFloat: Return the value as a float
	float		GetFloat(t_Str szKey, t_Str szSection, float def = 0.0) const;

	double 		GetDouble(t_Str szKey, t_Str szSection, double def = 0.0) const;

				// GetInt: Return the value as an int
	int			GetInt(t_Str szKey, t_Str szSection, int def = 0) const;
				// GetBool: Return the value as a bool
	bool		GetBool(t_Str szKey, t_Str szSection, bool def = true) const;

				// SetValue: Sets the value of a given key. Will create the
				// key if it is not found and AUTOCREATE_KEYS is active.
	bool		SetValue(t_Str szKey, t_Str szValue,
						 t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));

				// SetFloat: Sets the value of a given key. Will create the
				// key if it is not found and AUTOCREATE_KEYS is active.
	bool		SetFloat(t_Str szKey, float fValue,
						 t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));

	bool		SetDouble(t_Str szKey, double fValue,
						 t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));

				// SetInt: Sets the value of a given key. Will create the
				// key if it is not found and AUTOCREATE_KEYS is active.
	bool		SetInt(t_Str szKey, int nValue,
						 t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));

				// SetBool: Sets the value of a given key. Will create the
				// key if it is not found and AUTOCREATE_KEYS is active.
	bool		SetBool(t_Str szKey, bool bValue,
						 t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));

				// Sets the comment for a given key.
	bool		SetKeyComment(t_Str szKey, t_Str szComment, t_Str szSection = t_Str(""));

				// Sets the comment for a given section
	bool		SetSectionComment(t_Str szSection, t_Str szComment);

				// DeleteKey: Deletes a given key from a specific section
	bool		DeleteKey(t_Str szKey, t_Str szFromSection = t_Str(""));

				// DeleteSection: Deletes a given section.
	bool		DeleteSection(t_Str szSection);

				// Key/Section handling methods
				/////////////////////////////////////////////////////////////////

				// CreateKey: Creates a new key in the requested section. The
	            // Section will be created if it does not exist and the 
				// AUTOCREATE_SECTIONS bit is set.
	bool		CreateKey(t_Str szKey, t_Str szValue, 
		                  t_Str szComment = t_Str(""), t_Str szSection = t_Str(""));
				// CreateSection: Creates the new section if it does not allready
				// exist. Section is created with no keys.
	bool		CreateSection(t_Str szSection, t_Str szComment = t_Str(""));
				// CreateSection: Creates the new section if it does not allready
				// exist, and copies the keys passed into it into the new section.
	bool		CreateSection(t_Str szSection, t_Str szComment, KeyList Keys);

				// Utility Methods
				/////////////////////////////////////////////////////////////////
				// SectionCount: Returns the number of valid sections in the database.
	int			SectionCount();
				// KeyCount: Returns the total number of keys, across all sections.
	int			KeyCount();
				// Clear: Initializes the member variables to their default states
	void		Clear();
				// SetFileName: For use when creating the object by hand
				// initializes the file name so that it can be later saved.
	void		SetFileName(t_Str szFileName);
				// CommentStr
				// Parses a string into a proper comment token/comment.
	t_Str		CommentStr(t_Str szComment);
	
	bool		SectionExist(t_Str szSection) const {if (GetSection(szSection)) return true; else return false;}
	
	bool		KeyExist(t_Str szKey, t_Str szSection) const {if (GetKey(szKey, szSection) ) return true; else return false;}


protected:
				// Note: I've tried to insulate the end user from the internal
				// data structures as much as possible. This is by design. Doing
				// so has caused some performance issues (multiple calls to a
				// GetSection() function that would otherwise not be necessary,etc).
				// But, I believe that doing so will provide a safer, more stable
				// environment. You'll notice that nothing returns a reference,
				// to modify the data values, you have to call member functions.
				// think carefully before changing this.

				// GetKey: Returns the requested key (if found) from the requested
				// Section. Returns NULL otherwise.
	t_Key*		GetKey(t_Str szKey, t_Str szSection) const;
				// GetSection: Returns the requested section (if found), NULL otherwise.
	t_Section*	GetSection(t_Str szSection) const;


// Data
public:
	long		m_Flags;		// Our settings flags.

protected:
	SectionList	m_Sections;		// Our list of sections
	t_Str		m_szFileName;	// The filename to write to
	bool		m_bDirty;		// Tracks whether or not data has changed.
	bool		m_bShowErrors;  // Show error messages for mismatched keys
};


} // namespace

#endif
