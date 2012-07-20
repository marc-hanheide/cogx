/* A Bison parser, made by GNU Bison 1.875.  */

/* Skeleton parser for Yacc-like parsing with Bison,
   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002 Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

/* As a special exception, when this file is copied by Bison into a
   Bison output file, you may use that output file without restriction.
   This special exception was added by the Free Software Foundation
   in version 1.24 of Bison.  */

/* Written by Richard Stallman by simplifying the original so called
   ``semantic'' parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0

/* If NAME_PREFIX is specified substitute the variables and functions
   names.  */
#define yyparse ops_pddlparse
#define yylex   ops_pddllex
#define yyerror ops_pddlerror
#define yylval  ops_pddllval
#define yychar  ops_pddlchar
#define yydebug ops_pddldebug
#define yynerrs ops_pddlnerrs


/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     DEFINE_TOK = 258,
     DOMAIN_TOK = 259,
     REQUIREMENTS_TOK = 260,
     TYPES_TOK = 261,
     EITHER_TOK = 262,
     CONSTANTS_TOK = 263,
     PREDICATES_TOK = 264,
     ACTION_TOK = 265,
     AXIOM_TOK = 266,
     VARS_TOK = 267,
     IMPLIES_TOK = 268,
     PRECONDITION_TOK = 269,
     REPLAN_CONDITION_TOK = 270,
     PARAMETERS_TOK = 271,
     EFFECT_TOK = 272,
     EQ_TOK = 273,
     AND_TOK = 274,
     NOT_TOK = 275,
     WHEN_TOK = 276,
     FORALL_TOK = 277,
     IMPLY_TOK = 278,
     OR_TOK = 279,
     EXISTS_TOK = 280,
     NAME = 281,
     VARIABLE = 282,
     TYPE = 283,
     OPEN_PAREN = 284,
     CLOSE_PAREN = 285
   };
#endif
#define DEFINE_TOK 258
#define DOMAIN_TOK 259
#define REQUIREMENTS_TOK 260
#define TYPES_TOK 261
#define EITHER_TOK 262
#define CONSTANTS_TOK 263
#define PREDICATES_TOK 264
#define ACTION_TOK 265
#define AXIOM_TOK 266
#define VARS_TOK 267
#define IMPLIES_TOK 268
#define PRECONDITION_TOK 269
#define REPLAN_CONDITION_TOK 270
#define PARAMETERS_TOK 271
#define EFFECT_TOK 272
#define EQ_TOK 273
#define AND_TOK 274
#define NOT_TOK 275
#define WHEN_TOK 276
#define FORALL_TOK 277
#define IMPLY_TOK 278
#define OR_TOK 279
#define EXISTS_TOK 280
#define NAME 281
#define VARIABLE 282
#define TYPE 283
#define OPEN_PAREN 284
#define CLOSE_PAREN 285




/* Copy the first part of user declarations.  */
#line 1 "scan-ops_pddl.y"

#ifdef YYDEBUG
  extern int yydebug=1;
#endif


#include <stdio.h>
#include <string.h> 
#include "ff.h"
#include "memory.h"
#include "parse.h"

#ifndef YYMAXDEPTH
#define YYMAXDEPTH 10000000
#endif


#ifndef SCAN_ERR
#define SCAN_ERR
#define DOMDEF_EXPECTED            0
#define DOMAIN_EXPECTED            1
#define DOMNAME_EXPECTED           2
#define LBRACKET_EXPECTED          3
#define RBRACKET_EXPECTED          4
#define DOMDEFS_EXPECTED           5
#define REQUIREM_EXPECTED          6
#define TYPEDLIST_EXPECTED         7
#define LITERAL_EXPECTED           8
#define PRECONDDEF_UNCORRECT       9
#define TYPEDEF_EXPECTED          10
#define CONSTLIST_EXPECTED        11
#define PREDDEF_EXPECTED          12 
#define NAME_EXPECTED             13
#define VARIABLE_EXPECTED         14
#define ACTIONFUNCTOR_EXPECTED    15
#define ATOM_FORMULA_EXPECTED     16
#define EFFECT_DEF_EXPECTED       17
#define NEG_FORMULA_EXPECTED      18
#define NOT_SUPPORTED             19
#define ACTION                    20
#endif


#define NAME_STR "name\0"
#define VARIABLE_STR "variable\0"
#define STANDARD_TYPE "OBJECT\0"
 

static char *serrmsg[] = {
  "domain definition expected",
  "'domain' expected",
  "domain name expected",
  "'(' expected",
  "')' expected",
  "additional domain definitions expected",
  "requirements (e.g. ':STRIPS') expected",
  "typed list of <%s> expected",
  "literal expected",
  "uncorrect precondition definition",
  "type definition expected",
  "list of constants expected",
  "predicate definition expected",
  "<name> expected",
  "<variable> expected",
  "action functor expected",
  "atomic formula expected",
  "effect definition expected",
  "negated atomic formula expected",
  "requirement %s not supported by this IPP version",  
  "action definition is not correct",
  NULL
};


/* void opserr( int errno, char *par ); */


static int sact_err;
static char *sact_err_par = NULL;
static PlOperator *scur_op = NULL;
static Bool sis_negated = FALSE;
static Bool parsing_replan_conds = FALSE;


int supported( char *str )

{

  int i;
  char * sup[] = { ":STRIPS", ":NEGATION", ":NEGATIVE-PRECONDITIONS", ":EQUALITY",":TYPING", 
		   ":CONDITIONAL-EFFECTS", ":DISJUNCTIVE-PRECONDITIONS", 
		   ":EXISTENTIAL-PRECONDITIONS", ":UNIVERSAL-PRECONDITIONS", 
		   ":QUANTIFIED-PRECONDITIONS", ":ADL", ":DERIVED-PREDICATES", 
		   NULL };     

  for (i=0; NULL != sup[i]; i++) {
    if ( SAME == strcmp(sup[i], str) ) {
      return TRUE;
    }
  }
  
  return FALSE;

}



/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 112 "scan-ops_pddl.y"
typedef union YYSTYPE {

  char string[MAX_LENGTH];
  char *pstring;
  PlNode *pPlNode;
  FactList *pFactList;
  TokenList *pTokenList;
  TypedList *pTypedList;

} YYSTYPE;
/* Line 191 of yacc.c.  */
#line 262 "scan-ops_pddl.tab.c"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 214 of yacc.c.  */
#line 274 "scan-ops_pddl.tab.c"

#if ! defined (yyoverflow) || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# if YYSTACK_USE_ALLOCA
#  define YYSTACK_ALLOC alloca
# else
#  ifndef YYSTACK_USE_ALLOCA
#   if defined (alloca) || defined (_ALLOCA_H)
#    define YYSTACK_ALLOC alloca
#   else
#    ifdef __GNUC__
#     define YYSTACK_ALLOC __builtin_alloca
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning. */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
# else
#  if defined (__STDC__) || defined (__cplusplus)
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   define YYSIZE_T size_t
#  endif
#  define YYSTACK_ALLOC malloc
#  define YYSTACK_FREE free
# endif
#endif /* ! defined (yyoverflow) || YYERROR_VERBOSE */


#if (! defined (yyoverflow) \
     && (! defined (__cplusplus) \
	 || (YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  short yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (short) + sizeof (YYSTYPE))				\
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  register YYSIZE_T yyi;		\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (0)
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack, Stack, yysize);				\
	Stack = &yyptr->Stack;						\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (0)

#endif

#if defined (__STDC__) || defined (__cplusplus)
   typedef signed char yysigned_char;
#else
   typedef short yysigned_char;
#endif

/* YYFINAL -- State number of the termination state. */
#define YYFINAL  3
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   155

/* YYNTOKENS -- Number of terminals. */
#define YYNTOKENS  31
/* YYNNTS -- Number of nonterminals. */
#define YYNNTS  42
/* YYNRULES -- Number of rules. */
#define YYNRULES  77
/* YYNRULES -- Number of states. */
#define YYNSTATES  172

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   285

#define YYTRANSLATE(YYX) 						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const unsigned char yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const unsigned short yyprhs[] =
{
       0,     0,     3,     4,     7,     8,    14,    19,    21,    24,
      27,    30,    33,    36,    39,    40,    46,    47,    48,    55,
      56,    57,    65,    66,    67,    71,    72,    78,    79,    85,
      86,    87,    96,   105,   106,   111,   112,   118,   119,   124,
     125,   126,   132,   133,   138,   140,   145,   150,   155,   161,
     169,   177,   178,   181,   183,   188,   196,   202,   203,   206,
     211,   213,   218,   223,   224,   227,   229,   231,   233,   236,
     238,   239,   245,   249,   252,   253,   259,   263
};

/* YYRHS -- A `-1'-separated list of the rules' RHS. */
static const yysigned_char yyrhs[] =
{
      32,     0,    -1,    -1,    33,    34,    -1,    -1,    29,     3,
      36,    35,    37,    -1,    29,     4,    26,    30,    -1,    30,
      -1,    42,    37,    -1,    49,    37,    -1,    47,    37,    -1,
      51,    37,    -1,    54,    37,    -1,    38,    37,    -1,    -1,
      29,     9,    40,    39,    30,    -1,    -1,    -1,    29,    26,
      72,    30,    41,    40,    -1,    -1,    -1,    29,     5,    43,
      26,    44,    45,    30,    -1,    -1,    -1,    26,    46,    45,
      -1,    -1,    29,     6,    48,    71,    30,    -1,    -1,    29,
       8,    50,    71,    30,    -1,    -1,    -1,    29,    10,    52,
      26,    53,    55,    56,    30,    -1,    29,    11,    29,    70,
      72,    30,    61,    30,    -1,    -1,    16,    29,    72,    30,
      -1,    -1,    12,    29,    72,    30,    56,    -1,    -1,    14,
      61,    57,    56,    -1,    -1,    -1,    15,    58,    61,    59,
      56,    -1,    -1,    17,    63,    60,    56,    -1,    65,    -1,
      29,    19,    62,    30,    -1,    29,    24,    62,    30,    -1,
      29,    20,    61,    30,    -1,    29,    23,    61,    61,    30,
      -1,    29,    25,    29,    72,    30,    61,    30,    -1,    29,
      22,    29,    72,    30,    61,    30,    -1,    -1,    61,    62,
      -1,    65,    -1,    29,    19,    64,    30,    -1,    29,    22,
      29,    72,    30,    63,    30,    -1,    29,    21,    61,    63,
      30,    -1,    -1,    63,    64,    -1,    29,    20,    66,    30,
      -1,    66,    -1,    29,    70,    67,    30,    -1,    29,    18,
      67,    30,    -1,    -1,    68,    67,    -1,    26,    -1,    27,
      -1,    26,    -1,    26,    69,    -1,    26,    -1,    -1,    26,
       7,    69,    30,    71,    -1,    26,    28,    71,    -1,    26,
      71,    -1,    -1,    27,     7,    69,    30,    72,    -1,    27,
      28,    72,    -1,    27,    72,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const unsigned short yyrline[] =
{
       0,   170,   170,   170,   181,   180,   194,   204,   206,   208,
     210,   212,   214,   216,   223,   222,   232,   235,   234,   264,
     268,   263,   279,   283,   282,   296,   295,   309,   308,   324,
     328,   323,   344,   387,   391,   405,   408,   427,   426,   433,
     437,   432,   447,   446,   461,   476,   482,   488,   494,   504,
     519,   539,   543,   556,   569,   575,   590,   609,   613,   625,
     631,   640,   647,   660,   662,   673,   679,   689,   696,   708,
     719,   721,   731,   742,   762,   764,   773,   784
};
#endif

#if YYDEBUG || YYERROR_VERBOSE
/* YYTNME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals. */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "DEFINE_TOK", "DOMAIN_TOK", 
  "REQUIREMENTS_TOK", "TYPES_TOK", "EITHER_TOK", "CONSTANTS_TOK", 
  "PREDICATES_TOK", "ACTION_TOK", "AXIOM_TOK", "VARS_TOK", "IMPLIES_TOK", 
  "PRECONDITION_TOK", "REPLAN_CONDITION_TOK", "PARAMETERS_TOK", 
  "EFFECT_TOK", "EQ_TOK", "AND_TOK", "NOT_TOK", "WHEN_TOK", "FORALL_TOK", 
  "IMPLY_TOK", "OR_TOK", "EXISTS_TOK", "NAME", "VARIABLE", "TYPE", 
  "OPEN_PAREN", "CLOSE_PAREN", "$accept", "file", "@1", 
  "domain_definition", "@2", "domain_name", "optional_domain_defs", 
  "predicates_def", "@3", "predicates_list", "@4", "require_def", "@5", 
  "@6", "require_key_star", "@7", "types_def", "@8", "constants_def", 
  "@9", "action_def", "@10", "@11", "axiom_def", "param_def", 
  "action_def_body", "@12", "@13", "@14", "@15", "adl_goal_description", 
  "adl_goal_description_star", "adl_effect", "adl_effect_star", 
  "literal_term", "atomic_formula_term", "term_star", "term", "name_plus", 
  "predicate", "typed_list_name", "typed_list_variable", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const unsigned short yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const unsigned char yyr1[] =
{
       0,    31,    33,    32,    35,    34,    36,    37,    37,    37,
      37,    37,    37,    37,    39,    38,    40,    41,    40,    43,
      44,    42,    45,    46,    45,    48,    47,    50,    49,    52,
      53,    51,    54,    55,    55,    56,    56,    57,    56,    58,
      59,    56,    60,    56,    61,    61,    61,    61,    61,    61,
      61,    62,    62,    63,    63,    63,    63,    64,    64,    65,
      65,    66,    66,    67,    67,    68,    68,    69,    69,    70,
      71,    71,    71,    71,    72,    72,    72,    72
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const unsigned char yyr2[] =
{
       0,     2,     0,     2,     0,     5,     4,     1,     2,     2,
       2,     2,     2,     2,     0,     5,     0,     0,     6,     0,
       0,     7,     0,     0,     3,     0,     5,     0,     5,     0,
       0,     8,     8,     0,     4,     0,     5,     0,     4,     0,
       0,     5,     0,     4,     1,     4,     4,     4,     5,     7,
       7,     0,     2,     1,     4,     7,     5,     0,     2,     4,
       1,     4,     4,     0,     2,     1,     1,     1,     2,     1,
       0,     5,     3,     2,     0,     5,     3,     2
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const unsigned char yydefact[] =
{
       2,     0,     0,     1,     0,     3,     0,     0,     4,     0,
       0,     0,     0,     7,     5,     0,     0,     0,     0,     0,
       0,     6,    19,    25,    27,    16,    29,     0,    13,     8,
      10,     9,    11,    12,     0,    70,    70,     0,    14,     0,
       0,    20,    70,     0,     0,    74,     0,    30,    69,    74,
      22,     0,    70,    73,    26,    28,    74,     0,    15,    33,
       0,    23,     0,    67,     0,    72,     0,    74,    77,    17,
       0,    35,     0,    22,    21,    68,    70,     0,    76,    16,
      74,     0,     0,    39,     0,     0,     0,     0,    44,    60,
      24,    71,    74,    18,     0,    74,    37,     0,     0,    42,
      53,    31,    63,    51,     0,     0,     0,    51,     0,    63,
      32,    75,    34,     0,    35,    40,    57,     0,     0,     0,
      35,    65,    66,     0,    63,    51,     0,     0,     0,    74,
       0,     0,    74,     0,    35,    38,    35,    57,     0,     0,
       0,     0,    74,    43,    62,    64,    52,    45,    47,    59,
       0,     0,    46,     0,    61,    36,    41,    58,    54,     0,
       0,     0,    48,     0,    56,     0,     0,     0,     0,    50,
      49,    55
};

/* YYDEFGOTO[NTERM-NUM]. */
static const short yydefgoto[] =
{
      -1,     1,     2,     5,    10,     8,    14,    15,    46,    38,
      79,    16,    34,    50,    62,    73,    17,    35,    18,    36,
      19,    39,    59,    20,    71,    85,   114,    97,   136,   120,
     125,   126,   137,   138,    88,    89,   123,   124,    64,   109,
      43,    57
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -105
static const yysigned_char yypact[] =
{
    -105,    15,    -9,  -105,    26,  -105,    12,    40,  -105,    19,
     -24,    25,    66,  -105,  -105,   -24,   -24,   -24,   -24,   -24,
     -24,  -105,  -105,  -105,  -105,    20,  -105,    27,  -105,  -105,
    -105,  -105,  -105,  -105,    39,    47,    47,    52,  -105,    53,
      55,  -105,    -5,    57,    70,    75,    71,  -105,  -105,    75,
      56,    77,    47,  -105,  -105,  -105,    10,    76,  -105,    88,
      78,  -105,    80,    77,    81,  -105,    77,    75,  -105,  -105,
      83,    74,    84,    56,  -105,  -105,    47,    85,  -105,    20,
      75,    87,    84,  -105,    89,    90,    44,    91,  -105,  -105,
    -105,  -105,    75,  -105,    92,    75,  -105,    84,    32,  -105,
    -105,  -105,    13,    84,    84,    94,    84,    84,    95,    13,
    -105,  -105,  -105,    96,    74,  -105,    89,    98,    84,    99,
      74,  -105,  -105,   100,    13,    84,   101,   102,   103,    75,
      84,   104,    75,   105,    74,  -105,    74,    89,   106,    16,
     103,    89,    75,  -105,  -105,  -105,  -105,  -105,  -105,  -105,
     107,   108,  -105,   109,  -105,  -105,  -105,  -105,  -105,   110,
     111,    84,  -105,    84,  -105,    89,   112,   113,   114,  -105,
    -105,  -105
};

/* YYPGOTO[NTERM-NUM].  */
static const yysigned_char yypgoto[] =
{
    -105,  -105,  -105,  -105,  -105,  -105,    79,  -105,  -105,    28,
    -105,  -105,  -105,  -105,    36,  -105,  -105,  -105,  -105,  -105,
    -105,  -105,  -105,  -105,  -105,  -104,  -105,  -105,  -105,  -105,
     -71,   -98,   -81,   -32,   -80,   -92,   -96,  -105,   -44,   115,
     -28,   -49
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -1
static const unsigned char yytable[] =
{
      60,    87,    51,    99,   100,    12,    13,    68,    44,   131,
     135,    96,   128,   133,    53,     3,   143,    66,    78,    75,
       4,    42,    77,    52,    65,   140,   115,   146,   145,     6,
     155,    94,   156,   127,   102,   130,   100,    56,    67,   121,
     122,     7,    48,   111,     9,    11,   113,   141,    91,    37,
     102,   116,   117,   118,   119,    21,    40,   100,    48,   151,
     159,   100,   102,   103,   104,    41,   105,   106,   107,   108,
      48,    22,    23,    42,    24,    25,    26,    27,    45,    47,
     150,    48,    61,   153,   168,   100,    81,    54,    82,    83,
     166,    84,   167,   160,    28,    29,    30,    31,    32,    33,
      55,    58,    56,    63,    70,   157,    69,    93,    72,    90,
      74,    76,    80,    86,     0,    92,    95,     0,    98,     0,
     101,   110,   112,   129,   132,     0,   134,   139,   142,     0,
     144,   147,   148,   149,   152,   154,   158,   161,   162,   163,
     164,   165,   169,   170,   171,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,    49
};

static const short yycheck[] =
{
      49,    72,     7,    84,    84,    29,    30,    56,    36,   107,
     114,    82,   104,   109,    42,     0,   120,     7,    67,    63,
      29,    26,    66,    28,    52,   117,    97,   125,   124,     3,
     134,    80,   136,   104,    18,   106,   116,    27,    28,    26,
      27,    29,    26,    92,     4,    26,    95,   118,    76,    29,
      18,    19,    20,    21,    22,    30,    29,   137,    26,   130,
     141,   141,    18,    19,    20,    26,    22,    23,    24,    25,
      26,     5,     6,    26,     8,     9,    10,    11,    26,    26,
     129,    26,    26,   132,   165,   165,    12,    30,    14,    15,
     161,    17,   163,   142,    15,    16,    17,    18,    19,    20,
      30,    30,    27,    26,    16,   137,    30,    79,    30,    73,
      30,    30,    29,    29,    -1,    30,    29,    -1,    29,    -1,
      30,    30,    30,    29,    29,    -1,    30,    29,    29,    -1,
      30,    30,    30,    30,    30,    30,    30,    30,    30,    30,
      30,    30,    30,    30,    30,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    -1,    -1,    40
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const unsigned char yystos[] =
{
       0,    32,    33,     0,    29,    34,     3,    29,    36,     4,
      35,    26,    29,    30,    37,    38,    42,    47,    49,    51,
      54,    30,     5,     6,     8,     9,    10,    11,    37,    37,
      37,    37,    37,    37,    43,    48,    50,    29,    40,    52,
      29,    26,    26,    71,    71,    26,    39,    26,    26,    70,
      44,     7,    28,    71,    30,    30,    27,    72,    30,    53,
      72,    26,    45,    26,    69,    71,     7,    28,    72,    30,
      16,    55,    30,    46,    30,    69,    30,    69,    72,    41,
      29,    12,    14,    15,    17,    56,    29,    61,    65,    66,
      45,    71,    30,    40,    72,    29,    61,    58,    29,    63,
      65,    30,    18,    19,    20,    22,    23,    24,    25,    70,
      30,    72,    30,    72,    57,    61,    19,    20,    21,    22,
      60,    26,    27,    67,    68,    61,    62,    61,    66,    29,
      61,    62,    29,    67,    30,    56,    59,    63,    64,    29,
      66,    61,    29,    56,    30,    67,    62,    30,    30,    30,
      72,    61,    30,    72,    30,    56,    56,    64,    30,    63,
      72,    30,    30,    30,    30,    30,    61,    61,    63,    30,
      30,    30
};

#if ! defined (YYSIZE_T) && defined (__SIZE_TYPE__)
# define YYSIZE_T __SIZE_TYPE__
#endif
#if ! defined (YYSIZE_T) && defined (size_t)
# define YYSIZE_T size_t
#endif
#if ! defined (YYSIZE_T)
# if defined (__STDC__) || defined (__cplusplus)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# endif
#endif
#if ! defined (YYSIZE_T)
# define YYSIZE_T unsigned int
#endif

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrlab1

/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK;						\
      goto yybackup;						\
    }								\
  else								\
    { 								\
      yyerror ("syntax error: cannot back up");\
      YYERROR;							\
    }								\
while (0)

#define YYTERROR	1
#define YYERRCODE	256

/* YYLLOC_DEFAULT -- Compute the default location (before the actions
   are run).  */

#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)         \
  Current.first_line   = Rhs[1].first_line;      \
  Current.first_column = Rhs[1].first_column;    \
  Current.last_line    = Rhs[N].last_line;       \
  Current.last_column  = Rhs[N].last_column;
#endif

/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (0)

# define YYDSYMPRINT(Args)			\
do {						\
  if (yydebug)					\
    yysymprint Args;				\
} while (0)

# define YYDSYMPRINTF(Title, Token, Value, Location)		\
do {								\
  if (yydebug)							\
    {								\
      YYFPRINTF (stderr, "%s ", Title);				\
      yysymprint (stderr, 					\
                  Token, Value);	\
      YYFPRINTF (stderr, "\n");					\
    }								\
} while (0)

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (cinluded).                                                   |
`------------------------------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yy_stack_print (short *bottom, short *top)
#else
static void
yy_stack_print (bottom, top)
    short *bottom;
    short *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (/* Nothing. */; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yy_reduce_print (int yyrule)
#else
static void
yy_reduce_print (yyrule)
    int yyrule;
#endif
{
  int yyi;
  unsigned int yylineno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %u), ",
             yyrule - 1, yylineno);
  /* Print the symbols being reduced, and their result.  */
  for (yyi = yyprhs[yyrule]; 0 <= yyrhs[yyi]; yyi++)
    YYFPRINTF (stderr, "%s ", yytname [yyrhs[yyi]]);
  YYFPRINTF (stderr, "-> %s\n", yytname [yyr1[yyrule]]);
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (Rule);		\
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YYDSYMPRINT(Args)
# define YYDSYMPRINTF(Title, Token, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   SIZE_MAX < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#if YYMAXDEPTH == 0
# undef YYMAXDEPTH
#endif

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined (__GLIBC__) && defined (_STRING_H)
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
#   if defined (__STDC__) || defined (__cplusplus)
yystrlen (const char *yystr)
#   else
yystrlen (yystr)
     const char *yystr;
#   endif
{
  register const char *yys = yystr;

  while (*yys++ != '\0')
    continue;

  return yys - yystr - 1;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined (__GLIBC__) && defined (_STRING_H) && defined (_GNU_SOURCE)
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
#   if defined (__STDC__) || defined (__cplusplus)
yystpcpy (char *yydest, const char *yysrc)
#   else
yystpcpy (yydest, yysrc)
     char *yydest;
     const char *yysrc;
#   endif
{
  register char *yyd = yydest;
  register const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

#endif /* !YYERROR_VERBOSE */



#if YYDEBUG
/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yysymprint (FILE *yyoutput, int yytype, YYSTYPE *yyvaluep)
#else
static void
yysymprint (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  /* Pacify ``unused variable'' warnings.  */
  (void) yyvaluep;

  if (yytype < YYNTOKENS)
    {
      YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
# ifdef YYPRINT
      YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
    }
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  switch (yytype)
    {
      default:
        break;
    }
  YYFPRINTF (yyoutput, ")");
}

#endif /* ! YYDEBUG */
/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yydestruct (int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yytype, yyvaluep)
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  /* Pacify ``unused variable'' warnings.  */
  (void) yyvaluep;

  switch (yytype)
    {

      default:
        break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
# if defined (__STDC__) || defined (__cplusplus)
int yyparse (void *YYPARSE_PARAM);
# else
int yyparse ();
# endif
#else /* ! YYPARSE_PARAM */
#if defined (__STDC__) || defined (__cplusplus)
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
# if defined (__STDC__) || defined (__cplusplus)
int yyparse (void *YYPARSE_PARAM)
# else
int yyparse (YYPARSE_PARAM)
  void *YYPARSE_PARAM;
# endif
#else /* ! YYPARSE_PARAM */
#if defined (__STDC__) || defined (__cplusplus)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  register int yystate;
  register int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  short	yyssa[YYINITDEPTH];
  short *yyss = yyssa;
  register short *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  register YYSTYPE *yyvsp;



#define YYPOPSTACK   (yyvsp--, yyssp--)

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* When reducing, the number of symbols on the RHS of the reduced
     rule.  */
  int yylen;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed. so pushing a state here evens the stacks.
     */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack. Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	short *yyss1 = yyss;


	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow ("parser stack overflow",
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),

		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyoverflowlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyoverflowlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	short *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyoverflowlab;
	YYSTACK_RELOCATE (yyss);
	YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;


      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

/* Do appropriate processing given the current state.  */
/* Read a lookahead token if we need one and don't already have one.  */
/* yyresume: */

  /* First try to decide what to do without reference to lookahead token.  */

  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YYDSYMPRINTF ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Shift the lookahead token.  */
  YYDPRINTF ((stderr, "Shifting token %s, ", yytname[yytoken]));

  /* Discard the token being shifted unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  *++yyvsp = yylval;


  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  yystate = yyn;
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 170 "scan-ops_pddl.y"
    { 
  opserr( DOMDEF_EXPECTED, NULL ); 
;}
    break;

  case 4:
#line 181 "scan-ops_pddl.y"
    { 
;}
    break;

  case 5:
#line 184 "scan-ops_pddl.y"
    {
  if ( gcmd_line.display_info >= 1 ) {
    printf("\ndomain '%s' defined\n", gdomain_name);
  }
;}
    break;

  case 6:
#line 195 "scan-ops_pddl.y"
    { 
  gdomain_name = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( gdomain_name, yyvsp[-1].string);
;}
    break;

  case 14:
#line 223 "scan-ops_pddl.y"
    {
;}
    break;

  case 15:
#line 226 "scan-ops_pddl.y"
    { 
;}
    break;

  case 16:
#line 232 "scan-ops_pddl.y"
    {;}
    break;

  case 17:
#line 235 "scan-ops_pddl.y"
    {

  TypedListList *tll;

  if ( gparse_predicates ) {
    tll = gparse_predicates;
    while ( tll->next ) {
      tll = tll->next;
    }
    tll->next = new_TypedListList();
    tll = tll->next;
  } else {
    tll = new_TypedListList();
    gparse_predicates = tll;
  }

  tll->predicate = new_Token( strlen( yyvsp[-2].string ) + 1);
  strcpy( tll->predicate, yyvsp[-2].string );

  tll->args = yyvsp[-1].pTypedList;

;}
    break;

  case 19:
#line 264 "scan-ops_pddl.y"
    { 
  opserr( REQUIREM_EXPECTED, NULL ); 
;}
    break;

  case 20:
#line 268 "scan-ops_pddl.y"
    { 
  if ( !supported( yyvsp[0].string ) ) {
    opserr( NOT_SUPPORTED, yyvsp[0].string );
    yyerror();
  }
;}
    break;

  case 23:
#line 283 "scan-ops_pddl.y"
    { 
  if ( !supported( yyvsp[0].string ) ) {
    opserr( NOT_SUPPORTED, yyvsp[0].string );
    yyerror();
  }
;}
    break;

  case 25:
#line 296 "scan-ops_pddl.y"
    { 
  opserr( TYPEDEF_EXPECTED, NULL ); 
;}
    break;

  case 26:
#line 300 "scan-ops_pddl.y"
    {
  gparse_types = yyvsp[-1].pTypedList;
;}
    break;

  case 27:
#line 309 "scan-ops_pddl.y"
    { 
  opserr( CONSTLIST_EXPECTED, NULL ); 
;}
    break;

  case 28:
#line 313 "scan-ops_pddl.y"
    {
  gparse_constants = yyvsp[-1].pTypedList;
;}
    break;

  case 29:
#line 324 "scan-ops_pddl.y"
    { 
  opserr( ACTION, NULL ); 
;}
    break;

  case 30:
#line 328 "scan-ops_pddl.y"
    { 
  scur_op = new_PlOperator( yyvsp[0].string );
  scur_op->axiom = FALSE;
;}
    break;

  case 31:
#line 333 "scan-ops_pddl.y"
    {
  scur_op->next = gloaded_ops;
  gloaded_ops = scur_op; 
;}
    break;

  case 32:
#line 345 "scan-ops_pddl.y"
    { 
  PlNode *pln;
  TypedList *tyl;
  TokenList *tmp, *prev;

  pln = new_PlNode(ATOM);
  tmp = new_TokenList();
  tmp->item = new_Token( strlen( yyvsp[-4].pstring )+1 ); 
  strcpy( tmp->item, yyvsp[-4].pstring );
  pln->atom = tmp;

  scur_op = new_PlOperator( "axiom" );
  scur_op->axiom = TRUE;
  scur_op->effects = pln;

  scur_op->preconds = yyvsp[-1].pPlNode;

  scur_op->params = NULL;
  scur_op->parse_params = yyvsp[-3].pTypedList;
  prev = pln->atom;
  for (tyl = scur_op->parse_params; tyl; tyl = tyl->next) {
    /* to be able to distinguish params from :VARS 
     */
    scur_op->number_of_real_params++;

    tmp = new_TokenList();
    tmp->item = new_Token( strlen( tyl->name )+1 ); 
    strcpy( tmp->item, tyl->name );
    prev->next = tmp;
    prev = tmp;
    
  }
  
  scur_op->next = gloaded_ops;
  gloaded_ops = scur_op; 
;}
    break;

  case 33:
#line 387 "scan-ops_pddl.y"
    { 
  scur_op->params = NULL; 
;}
    break;

  case 34:
#line 392 "scan-ops_pddl.y"
    {
  TypedList *tl;
  scur_op->parse_params = yyvsp[-1].pTypedList;
  for (tl = scur_op->parse_params; tl; tl = tl->next) {
    /* to be able to distinguish params from :VARS 
     */
    scur_op->number_of_real_params++;
  }
;}
    break;

  case 36:
#line 409 "scan-ops_pddl.y"
    {
  TypedList *tl = NULL;

  /* add vars as parameters 
   */
  if ( scur_op->parse_params ) {
    for( tl = scur_op->parse_params; tl->next; tl = tl->next ) {
      /* empty, get to the end of list 
       */
    }
    tl->next = yyvsp[-2].pTypedList;
    tl = tl->next;
  } else {
    scur_op->parse_params = yyvsp[-2].pTypedList;
  }
;}
    break;

  case 37:
#line 427 "scan-ops_pddl.y"
    { 
  scur_op->preconds = yyvsp[0].pPlNode; 
;}
    break;

  case 39:
#line 433 "scan-ops_pddl.y"
    {
  parsing_replan_conds = TRUE;
;}
    break;

  case 40:
#line 437 "scan-ops_pddl.y"
    { 
  PlNode* p = scur_op->preconds;
  scur_op->preconds = new_PlNode(AND);
  scur_op->preconds->sons = yyvsp[0].pPlNode;
  scur_op->preconds->sons->next = p;
  parsing_replan_conds = FALSE;
;}
    break;

  case 42:
#line 447 "scan-ops_pddl.y"
    { 
  scur_op->effects = yyvsp[0].pPlNode; 
;}
    break;

  case 44:
#line 462 "scan-ops_pddl.y"
    { 
  if ( sis_negated ) {
    yyval.pPlNode = new_PlNode(NOT);
    yyval.pPlNode->sons = new_PlNode(ATOM);
    yyval.pPlNode->sons->atom = yyvsp[0].pTokenList;
    yyval.pPlNode->sons->isReplanCond = parsing_replan_conds;
    sis_negated = FALSE;
  } else {
    yyval.pPlNode = new_PlNode(ATOM);
    yyval.pPlNode->atom = yyvsp[0].pTokenList;
    yyval.pPlNode->isReplanCond = parsing_replan_conds;
  }
;}
    break;

  case 45:
#line 477 "scan-ops_pddl.y"
    { 
  yyval.pPlNode = new_PlNode(AND);
  yyval.pPlNode->sons = yyvsp[-1].pPlNode;
;}
    break;

  case 46:
#line 483 "scan-ops_pddl.y"
    { 
  yyval.pPlNode = new_PlNode(OR);
  yyval.pPlNode->sons = yyvsp[-1].pPlNode;
;}
    break;

  case 47:
#line 489 "scan-ops_pddl.y"
    { 
  yyval.pPlNode = new_PlNode(NOT);
  yyval.pPlNode->sons = yyvsp[-1].pPlNode;
;}
    break;

  case 48:
#line 495 "scan-ops_pddl.y"
    { 
  PlNode *np = new_PlNode(NOT);
  np->sons = yyvsp[-2].pPlNode;
  np->next = yyvsp[-1].pPlNode;

  yyval.pPlNode = new_PlNode(OR);
  yyval.pPlNode->sons = np;
;}
    break;

  case 49:
#line 507 "scan-ops_pddl.y"
    { 

  PlNode *pln;

  pln = new_PlNode(EX);
  pln->parse_vars = yyvsp[-3].pTypedList;

  yyval.pPlNode = pln;
  pln->sons = yyvsp[-1].pPlNode;

;}
    break;

  case 50:
#line 522 "scan-ops_pddl.y"
    { 

  PlNode *pln;

  pln = new_PlNode(ALL);
  pln->parse_vars = yyvsp[-3].pTypedList;

  yyval.pPlNode = pln;
  pln->sons = yyvsp[-1].pPlNode;

;}
    break;

  case 51:
#line 539 "scan-ops_pddl.y"
    {
  yyval.pPlNode = NULL;
;}
    break;

  case 52:
#line 544 "scan-ops_pddl.y"
    {
  yyvsp[-1].pPlNode->next = yyvsp[0].pPlNode;
  yyval.pPlNode = yyvsp[-1].pPlNode;
;}
    break;

  case 53:
#line 557 "scan-ops_pddl.y"
    { 
  if ( sis_negated ) {
    yyval.pPlNode = new_PlNode(NOT);
    yyval.pPlNode->sons = new_PlNode(ATOM);
    yyval.pPlNode->sons->atom = yyvsp[0].pTokenList;
    sis_negated = FALSE;
  } else {
    yyval.pPlNode = new_PlNode(ATOM);
    yyval.pPlNode->atom = yyvsp[0].pTokenList;
  }
;}
    break;

  case 54:
#line 570 "scan-ops_pddl.y"
    { 
  yyval.pPlNode = new_PlNode(AND);
  yyval.pPlNode->sons = yyvsp[-1].pPlNode;
;}
    break;

  case 55:
#line 578 "scan-ops_pddl.y"
    { 

  PlNode *pln;

  pln = new_PlNode(ALL);
  pln->parse_vars = yyvsp[-3].pTypedList;

  yyval.pPlNode = pln;
  pln->sons = yyvsp[-1].pPlNode;

;}
    break;

  case 56:
#line 591 "scan-ops_pddl.y"
    {
  /* This will be conditional effects in FF representation, but here
   * a formula like (WHEN p q) will be saved as:
   *  [WHEN]
   *  [sons]
   *   /  \
   * [p]  [q]
   * That means, the first son is p, and the second one is q. 
   */
  yyval.pPlNode = new_PlNode(WHEN);
  yyvsp[-2].pPlNode->next = yyvsp[-1].pPlNode;
  yyval.pPlNode->sons = yyvsp[-2].pPlNode;
;}
    break;

  case 57:
#line 609 "scan-ops_pddl.y"
    { 
  yyval.pPlNode = NULL; 
;}
    break;

  case 58:
#line 614 "scan-ops_pddl.y"
    {
  yyvsp[-1].pPlNode->next = yyvsp[0].pPlNode;
  yyval.pPlNode = yyvsp[-1].pPlNode;
;}
    break;

  case 59:
#line 626 "scan-ops_pddl.y"
    { 
  yyval.pTokenList = yyvsp[-1].pTokenList;
  sis_negated = TRUE;
;}
    break;

  case 60:
#line 632 "scan-ops_pddl.y"
    {
  yyval.pTokenList = yyvsp[0].pTokenList;
;}
    break;

  case 61:
#line 641 "scan-ops_pddl.y"
    { 
  yyval.pTokenList = new_TokenList();
  yyval.pTokenList->item = yyvsp[-2].pstring;
  yyval.pTokenList->next = yyvsp[-1].pTokenList;
;}
    break;

  case 62:
#line 648 "scan-ops_pddl.y"
    {
  yyval.pTokenList = new_TokenList();
  yyval.pTokenList->item = new_Token( 5 );
  yyval.pTokenList->item = "=";
  yyval.pTokenList->next = yyvsp[-1].pTokenList;
;}
    break;

  case 63:
#line 660 "scan-ops_pddl.y"
    { yyval.pTokenList = NULL; ;}
    break;

  case 64:
#line 663 "scan-ops_pddl.y"
    {
  yyval.pTokenList = new_TokenList();
  yyval.pTokenList->item = yyvsp[-1].pstring;
  yyval.pTokenList->next = yyvsp[0].pTokenList;
;}
    break;

  case 65:
#line 674 "scan-ops_pddl.y"
    { 
  yyval.pstring = new_Token( strlen(yyvsp[0].string)+1 );
  strcpy( yyval.pstring, yyvsp[0].string );
;}
    break;

  case 66:
#line 680 "scan-ops_pddl.y"
    { 
  yyval.pstring = new_Token( strlen(yyvsp[0].string)+1 );
  strcpy( yyval.pstring, yyvsp[0].string );
;}
    break;

  case 67:
#line 690 "scan-ops_pddl.y"
    {
  yyval.pTokenList = new_TokenList();
  yyval.pTokenList->item = new_Token( strlen(yyvsp[0].string)+1 );
  strcpy( yyval.pTokenList->item, yyvsp[0].string );
;}
    break;

  case 68:
#line 697 "scan-ops_pddl.y"
    {
  yyval.pTokenList = new_TokenList();
  yyval.pTokenList->item = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( yyval.pTokenList->item, yyvsp[-1].string );
  yyval.pTokenList->next = yyvsp[0].pTokenList;
;}
    break;

  case 69:
#line 709 "scan-ops_pddl.y"
    { 
  yyval.pstring = new_Token( strlen(yyvsp[0].string)+1 );
  strcpy( yyval.pstring, yyvsp[0].string );
;}
    break;

  case 70:
#line 719 "scan-ops_pddl.y"
    { yyval.pTypedList = NULL; ;}
    break;

  case 71:
#line 722 "scan-ops_pddl.y"
    { 

  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-4].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-4].string );
  yyval.pTypedList->type = yyvsp[-2].pTokenList;
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;

  case 72:
#line 732 "scan-ops_pddl.y"
    {
  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-2].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-2].string );
  yyval.pTypedList->type = new_TokenList();
  yyval.pTypedList->type->item = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( yyval.pTypedList->type->item, yyvsp[-1].string );
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;

  case 73:
#line 743 "scan-ops_pddl.y"
    {
  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-1].string );
  if ( yyvsp[0].pTypedList ) {/* another element (already typed) is following */
    yyval.pTypedList->type = copy_TokenList( yyvsp[0].pTypedList->type );
  } else {/* no further element - it must be an untyped list */
    yyval.pTypedList->type = new_TokenList();
    yyval.pTypedList->type->item = new_Token( strlen(STANDARD_TYPE)+1 );
    strcpy( yyval.pTypedList->type->item, STANDARD_TYPE );
  }
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;

  case 74:
#line 762 "scan-ops_pddl.y"
    { yyval.pTypedList = NULL; ;}
    break;

  case 75:
#line 765 "scan-ops_pddl.y"
    { 
  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-4].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-4].string );
  yyval.pTypedList->type = yyvsp[-2].pTokenList;
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;

  case 76:
#line 774 "scan-ops_pddl.y"
    {
  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-2].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-2].string );
  yyval.pTypedList->type = new_TokenList();
  yyval.pTypedList->type->item = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( yyval.pTypedList->type->item, yyvsp[-1].string );
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;

  case 77:
#line 785 "scan-ops_pddl.y"
    {
  yyval.pTypedList = new_TypedList();
  yyval.pTypedList->name = new_Token( strlen(yyvsp[-1].string)+1 );
  strcpy( yyval.pTypedList->name, yyvsp[-1].string );
  if ( yyvsp[0].pTypedList ) {/* another element (already typed) is following */
    yyval.pTypedList->type = copy_TokenList( yyvsp[0].pTypedList->type );
  } else {/* no further element - it must be an untyped list */
    yyval.pTypedList->type = new_TokenList();
    yyval.pTypedList->type->item = new_Token( strlen(STANDARD_TYPE)+1 );
    strcpy( yyval.pTypedList->type->item, STANDARD_TYPE );
  }
  yyval.pTypedList->next = yyvsp[0].pTypedList;
;}
    break;


    }

/* Line 991 of yacc.c.  */
#line 1911 "scan-ops_pddl.tab.c"

  yyvsp -= yylen;
  yyssp -= yylen;


  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;


  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if YYERROR_VERBOSE
      yyn = yypact[yystate];

      if (YYPACT_NINF < yyn && yyn < YYLAST)
	{
	  YYSIZE_T yysize = 0;
	  int yytype = YYTRANSLATE (yychar);
	  char *yymsg;
	  int yyx, yycount;

	  yycount = 0;
	  /* Start YYX at -YYN if negative to avoid negative indexes in
	     YYCHECK.  */
	  for (yyx = yyn < 0 ? -yyn : 0;
	       yyx < (int) (sizeof (yytname) / sizeof (char *)); yyx++)
	    if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	      yysize += yystrlen (yytname[yyx]) + 15, yycount++;
	  yysize += yystrlen ("syntax error, unexpected ") + 1;
	  yysize += yystrlen (yytname[yytype]);
	  yymsg = (char *) YYSTACK_ALLOC (yysize);
	  if (yymsg != 0)
	    {
	      char *yyp = yystpcpy (yymsg, "syntax error, unexpected ");
	      yyp = yystpcpy (yyp, yytname[yytype]);

	      if (yycount < 5)
		{
		  yycount = 0;
		  for (yyx = yyn < 0 ? -yyn : 0;
		       yyx < (int) (sizeof (yytname) / sizeof (char *));
		       yyx++)
		    if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
		      {
			const char *yyq = ! yycount ? ", expecting " : " or ";
			yyp = yystpcpy (yyp, yyq);
			yyp = yystpcpy (yyp, yytname[yyx]);
			yycount++;
		      }
		}
	      yyerror (yymsg);
	      YYSTACK_FREE (yymsg);
	    }
	  else
	    yyerror ("syntax error; also virtual memory exhausted");
	}
      else
#endif /* YYERROR_VERBOSE */
	yyerror ("syntax error");
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      /* Return failure if at end of input.  */
      if (yychar == YYEOF)
        {
	  /* Pop the error token.  */
          YYPOPSTACK;
	  /* Pop the rest of the stack.  */
	  while (yyss < yyssp)
	    {
	      YYDSYMPRINTF ("Error: popping", yystos[*yyssp], yyvsp, yylsp);
	      yydestruct (yystos[*yyssp], yyvsp);
	      YYPOPSTACK;
	    }
	  YYABORT;
        }

      YYDSYMPRINTF ("Error: discarding", yytoken, &yylval, &yylloc);
      yydestruct (yytoken, &yylval);
      yychar = YYEMPTY;

    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab2;


/*----------------------------------------------------.
| yyerrlab1 -- error raised explicitly by an action.  |
`----------------------------------------------------*/
yyerrlab1:

  /* Suppress GCC warning that yyerrlab1 is unused when no action
     invokes YYERROR.  */
#if defined (__GNUC_MINOR__) && 2093 <= (__GNUC__ * 1000 + __GNUC_MINOR__) \
    && !defined __cplusplus
  __attribute__ ((__unused__))
#endif


  goto yyerrlab2;


/*---------------------------------------------------------------.
| yyerrlab2 -- pop states until the error token can be shifted.  |
`---------------------------------------------------------------*/
yyerrlab2:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;

      YYDSYMPRINTF ("Error: popping", yystos[*yyssp], yyvsp, yylsp);
      yydestruct (yystos[yystate], yyvsp);
      yyvsp--;
      yystate = *--yyssp;

      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  YYDPRINTF ((stderr, "Shifting error token, "));

  *++yyvsp = yylval;


  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#ifndef yyoverflow
/*----------------------------------------------.
| yyoverflowlab -- parser overflow comes here.  |
`----------------------------------------------*/
yyoverflowlab:
  yyerror ("parser stack overflow");
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
  return yyresult;
}


#line 802 "scan-ops_pddl.y"

#include "lex.ops_pddl.c"


/**********************************************************************
 * Functions
 **********************************************************************/

/* 
 * call	bison -pops -bscan-ops scan-ops.y
 */

void opserr( int errno, char *par )

{

/*   sact_err = errno; */

/*   if ( sact_err_par ) { */
/*     free(sact_err_par); */
/*   } */
/*   if ( par ) { */
/*     sact_err_par = new_Token(strlen(par)+1); */
/*     strcpy(sact_err_par, par); */
/*   } else { */
/*     sact_err_par = NULL; */
/*   } */

}
  


int yyerror( char *msg )

{

  fflush(stdout);
  fprintf(stderr, "\n%s: syntax error in line %d, '%s':\n", 
	  gact_filename, lineno, yytext);

  if ( NULL != sact_err_par ) {
    fprintf(stderr, "%s %s\n", serrmsg[sact_err], sact_err_par);
  } else {
    fprintf(stderr, "%s\n", serrmsg[sact_err]);
  }

  exit( 1 );

}



void load_ops_file( char *filename )

{

  FILE * fp;/* pointer to input files */
  char tmp[MAX_LENGTH] = "";

  /* open operator file 
   */
  if( ( fp = fopen( filename, "r" ) ) == NULL ) {
    sprintf(tmp, "\nff: can't find operator file: %s\n\n", filename );
    perror(tmp);
    exit( 1 );
  }

  gact_filename = filename;
  lineno = 1; 
  yyin = fp;

  yyparse();

  fclose( fp );/* and close file again */

}


