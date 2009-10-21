#include "StringToSlice.h"

using namespace std;
using namespace Abducer;

void
parseAndAddTermSeq(vector<Token *>::iterator & it, vector<TermPtr> & args)
{
	vector<Token *>::iterator orig = it;

	TermPtr arg = parseTerm(it);
	if (arg) {
		args.push_back(arg);

		if ((*it)->type() == Comma) {
			it++;
			parseAndAddTermSeq(it, args);
		}
	}
	else {
		// error in the child, abort too
		it = orig;
	}
}

void
parseAndAddModalitySeq(vector<Token *>::iterator & it, vector<ModalityPtr> & args)
{
	vector<Token *>::iterator orig = it;

	ModalityPtr arg = parseModality(it);
	if (arg) {
		args.push_back(arg);

		if ((*it)->type() == Comma) {
			it++;
			parseAndAddModalitySeq(it, args);
		}
	}
	else {
		// error in the child, abort too
		it = orig;
	}
}

Abducer::TermPtr
parseTerm(vector<Token *>::iterator & it)
{
	vector<Token *>::iterator orig = it;

	if ((*it)->type() == VariableName) {
		VariableNameToken * varTok = (VariableNameToken *) *it;
		VariableTermPtr vt = new VariableTerm();
		vt->type = Variable;
		vt->name = varTok->name();
		it++;
		return vt;
	}

	if ((*it)->type() == Atom) {
		AtomToken * atomTok = (AtomToken *) *it;
		FunctionTerm * ft = new FunctionTerm();
		ft->type = Function;
		ft->functor = atomTok->value();
		it++;
		if ((*it)->type() == OpenParenthesis) {
			it++;
			parseAndAddTermSeq(it, ft->args);

			if ((*it)->type() == CloseParenthesis) {
				// ok
				it++;
				return ft;
			}
			else {
				// closing parenthesis expected
				it = orig;
				return NULL;
			}
		}
		else {
			// no arguments apparently
			return ft;
		}
	}

	it = orig;
	return NULL;
}

Abducer::PredicatePtr
parsePredicate(vector<Token *>::iterator & it)
{
	vector<Token *>::iterator orig = it;
	PredicatePtr p = new Predicate();

	if ((*it)->type() == Atom) {
		AtomToken * psymTok = (AtomToken *) *it;
		p->predSym = psymTok->value();

		it++;
		if ((*it)->type() == OpenParenthesis) {
			it++;

			parseAndAddTermSeq(it, p->args);

			if ((*it)->type() == CloseParenthesis) {
				it++;
				if ((*it)->type() == Dot) {
					// ok
					it++;
					return p;
				}
				else {
					// syntax error, dot expected
					it = orig;
					return NULL;
				}
			}
			else {
				// closing parenthesis expected
				it = orig;
				return NULL;
			}
		}
		else if ((*it)->type() == Dot) {
			// ok
			it++;
			return p;
		}
		else {
			// syntax error, dot expected
			it = orig;
			return NULL;
		}
	}
	else {
		// syntax error, atom expected
		it = orig;
		return NULL;
	}
}

ModalisedFormulaPtr
parseModalisedFormula(vector<Token *>::iterator & it)
{
	vector<Token *>::iterator orig = it;

	ModalisedFormulaPtr mf = new ModalisedFormula();
	vector<ModalityPtr> mod;

	if ((*it)->type() == OpenCurlyBracket) {
		it++;

		parseAndAddModalitySeq(it, mod);

		if ((*it)->type() == CloseCurlyBracket) {
			// ok
			it++;
			mf->m = mod;
		}
		else {
			it = orig;
			return NULL;
		}
	}

	PredicatePtr p = parsePredicate(it);

	if (p) {
		mf->p = p;
		return mf;
	}
	else {
		it = orig;
		return NULL;
	}
}

Abducer::Agent
atomTokenToAgent(const AtomToken * tok)
{
	if (tok->value() == "h") {
		return human;
	}
	else {
		return robot;
	}
}

Abducer::ModalityPtr
parseModality(std::vector<Token *>::iterator & it)
{
	vector<Token *>::iterator orig = it;

	if ((*it)->type() == Atom) {
		AtomToken * atomTok = (AtomToken *) *it;
		if (atomTok->value() == string("i")) {
			it++;
			InfoModalityPtr im = new InfoModality();
			im->type = Info;
			return im;
		}
		else if (atomTok->value() == string("event")) {
			it++;
			EventModalityPtr em = new EventModality();
			em->type = Event;
			return em;
		}
		else if (atomTok->value() == string("intention")) {
			it++;
			IntentionModalityPtr im = new IntentionModality();
			im->type = Intention;
			return im;
		}
		else if (atomTok->value() == string("att")) {
			it++;
			AttStateModalityPtr am = new AttStateModality();
			am->type = AttState;
			return am;
		}
		else if (atomTok->value() == string("generate")) {
			it++;
			GenerationModalityPtr gm = new GenerationModality();
			gm->type = Generation;
			return gm;
		}
		else if (atomTok->value() == string("understand")) {
			it++;
			UnderstandingModalityPtr um = new UnderstandingModality();
			um->type = Understanding;
			return um;
		}
		else if (atomTok->value() == string("k")) {
			it++;
			KModalityPtr km = new KModality();
			km->type = K;
			if ((*it)->type() == OpenParenthesis) {
				it++;
				it++;  // skip the "now" atom
				it++;  // skip the comma
				if ((*it)->type() == Atom) {
					AtomToken * shareTok = (AtomToken *) *it;

					if (shareTok->value() == "private") {
						km->share = Private;
						it++;  // skip '('
						km->ag = atomTokenToAgent((AtomToken *)*it);
						km->ag2 = km->ag;
						it++;  // skip ')'
					}
					else if (shareTok->value() == "attrib") {
						km->share = Attribute;
						it++;  // skip '('
						km->ag = atomTokenToAgent((AtomToken *)*it);
						it++;  // skip ','
						km->ag2 = atomTokenToAgent((AtomToken *)*it);
						it++;  // skip ')'
					}
					else if (shareTok->value() == "mutual") {
						km->share = Mutual;
						it++;  // skip '('
						km->ag = atomTokenToAgent((AtomToken *)*it);
						it++;  // skip ','
						km->ag2 = atomTokenToAgent((AtomToken *)*it);
						it++;  // skip ')'
					}

					if ((*it)->type() == CloseParenthesis) {
						it++;
						return km;
					}
					else {
						it = orig;
						return NULL;
					}
				}
				else {
					it = orig;
					return NULL;
				}
			}
			else {
				it = orig;
				return NULL;
			}
		}
		else {
			it = orig;
			return NULL;
		}
	}
	else {
		it = orig;
		return NULL;
	}
}
