#ifndef TOKENS_H__
#define TOKENS_H__  1

#include <string>

enum TokenType {
	Comma,
	Dot,
	OpenParenthesis,
	CloseParenthesis,
	OpenBracket,
	CloseBracket,
	VariableName,
	Atom,
	Float,
	String
};

/*
 * Base class for tokens.
 */
class Token {
public:
	virtual std::string toMercuryString() = 0;
	virtual TokenType type() = 0;
};

// ','
class CommaToken : public Token {
public:
	//CommaToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// '.'
class DotToken : public Token {
public:
	//DotToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// '('
class OpenParenthesisToken : public Token {
public:
	//OpenParenthesisToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// ')'
class CloseParenthesisToken : public Token {
public:
	//CloseParenthesisToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// '['
class OpenBracketToken : public Token {
public:
	//OpenBracketToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// ']'
class CloseBracketToken : public Token {
public:
	//CloseBracketToken();
	virtual std::string toMercuryString();
	virtual TokenType type();
};

// variable name
class VariableNameToken : public Token {
public:
	VariableNameToken(const std::string & s);
	virtual std::string toMercuryString();
	virtual TokenType type();
	std::string name();  // unescaped name
protected:
	std::string nameValue;
};

// atom: either a sequence of [a-zA-Z0-9_] or a sequence of chars
// enclosed in apostrophes.
class AtomToken : public Token {
public:
	AtomToken(const std::string & s);
	virtual std::string toMercuryString();
	virtual TokenType type();
	std::string value();  // unescaped value
protected:
	std::string atomValue;
};

// float
class FloatToken : public Token {
public:
	FloatToken(double f);
	virtual std::string toMercuryString();
	virtual TokenType type();
	double value();  // unescaped value
protected:
	double floatValue;
};

// string
class StringToken : public Token {
public:
	StringToken(const std::string & s);
	virtual std::string toMercuryString();
	virtual TokenType type();
	std::string value();  // unescaped
protected:
	std::string stringValue;
};

#endif
