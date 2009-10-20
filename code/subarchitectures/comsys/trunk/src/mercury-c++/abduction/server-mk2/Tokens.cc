#include "Tokens.h"

#include <sstream>

using namespace std;

//-------------------------------------------------------------------------

string CommaToken::toMercuryString()
{
	return string(",");
}

TokenType CommaToken::type()
{
	return Comma;
}

//-------------------------------------------------------------------------

string DotToken::toMercuryString()
{
	return string(".");
}

TokenType DotToken::type()
{
	return Dot;
}

//-------------------------------------------------------------------------

string OpenParenthesisToken::toMercuryString()
{
	return string("(");
}

TokenType OpenParenthesisToken::type()
{
	return OpenParenthesis;
}

//-------------------------------------------------------------------------

string CloseParenthesisToken::toMercuryString()
{
	return string(")");
}

TokenType CloseParenthesisToken::type()
{
	return CloseParenthesis;
}

//-------------------------------------------------------------------------

string OpenBracketToken::toMercuryString()
{
	return string("[");
}

TokenType OpenBracketToken::type()
{
	return OpenBracket;
}

//-------------------------------------------------------------------------

string CloseBracketToken::toMercuryString()
{
	return string("]");
}

TokenType CloseBracketToken::type()
{
	return CloseBracket;
}

//-------------------------------------------------------------------------

VariableNameToken::VariableNameToken(const string & s)
: nameValue(s)
{ }

string VariableNameToken::toMercuryString()
{
	return nameValue;
}

TokenType VariableNameToken::type()
{
	return VariableName;
}

string VariableNameToken::name()
{
	return nameValue;
}

//-------------------------------------------------------------------------

AtomToken::AtomToken(const string & s)
: atomValue(s)
{ }

string AtomToken::toMercuryString()
{
	return atomValue;
}

TokenType AtomToken::type()
{
	return Atom;
}

string AtomToken::value()
{
	return atomValue;
}

//-------------------------------------------------------------------------

FloatToken::FloatToken(double f)
: floatValue(f)
{ }


string FloatToken::toMercuryString()
{
	stringstream ss;
	ss << floatValue;
	return ss.str();
}

TokenType FloatToken::type()
{
	return Float;
}

double FloatToken::value()
{
	return floatValue;
}

//-------------------------------------------------------------------------

StringToken::StringToken(const string & s)
: stringValue(s)
{ }

// FIXME: escapes!
string StringToken::toMercuryString()
{
	return string("\"" + stringValue + "\"");
}

TokenType StringToken::type()
{
	return String;
}

string StringToken::value()
{
	return stringValue;
}
