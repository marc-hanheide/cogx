/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.

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
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.3"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     OPEN_BRAC = 258,
     CLOSE_BRAC = 259,
     OPEN_SQ = 260,
     CLOSE_SQ = 261,
     DEFINE = 262,
     PDDLDOMAIN = 263,
     REQS = 264,
     EQUALITY = 265,
     STRIPS = 266,
     ADL = 267,
     NEGATIVE_PRECONDITIONS = 268,
     TYPING = 269,
     DISJUNCTIVE_PRECONDS = 270,
     EXT_PRECS = 271,
     UNIV_PRECS = 272,
     QUANT_PRECS = 273,
     COND_EFFS = 274,
     FLUENTS = 275,
     TIME = 276,
     DURATIVE_ACTIONS = 277,
     DURATION_INEQUALITIES = 278,
     CONTINUOUS_EFFECTS = 279,
     DERIVED_PREDICATES = 280,
     TIMED_INITIAL_LITERALS = 281,
     PREFERENCES = 282,
     CONSTRAINTS = 283,
     ACTION_COSTS = 284,
     ACTION = 285,
     NUMBER = 286,
     PROCESS = 287,
     EVENT = 288,
     DURATIVE_ACTION = 289,
     DERIVED = 290,
     CONSTANTS = 291,
     PREDS = 292,
     FUNCTIONS = 293,
     TYPES = 294,
     ARGS = 295,
     PRE = 296,
     CONDITION = 297,
     PREFERENCE = 298,
     START_PRE = 299,
     END_PRE = 300,
     EFFECTS = 301,
     INITIAL_EFFECT = 302,
     FINAL_EFFECT = 303,
     INVARIANT = 304,
     DURATION = 305,
     AT_START = 306,
     AT_END = 307,
     OVER_ALL = 308,
     AND = 309,
     OR = 310,
     EXISTS = 311,
     FORALL = 312,
     IMPLY = 313,
     NOT = 314,
     WHEN = 315,
     EITHER = 316,
     PROBLEM = 317,
     FORDOMAIN = 318,
     INITIALLY = 319,
     OBJECTS = 320,
     GOALS = 321,
     EQ = 322,
     LENGTH = 323,
     SERIAL = 324,
     PARALLEL = 325,
     METRIC = 326,
     MINIMIZE = 327,
     MAXIMIZE = 328,
     HASHT = 329,
     DURATION_VAR = 330,
     TOTAL_TIME = 331,
     INCREASE = 332,
     DECREASE = 333,
     SCALE_UP = 334,
     SCALE_DOWN = 335,
     ASSIGN = 336,
     GREATER = 337,
     GREATEQ = 338,
     LESS = 339,
     LESSEQ = 340,
     Q = 341,
     COLON = 342,
     ALWAYS = 343,
     SOMETIME = 344,
     WITHIN = 345,
     ATMOSTONCE = 346,
     SOMETIMEAFTER = 347,
     SOMETIMEBEFORE = 348,
     ALWAYSWITHIN = 349,
     HOLDDURING = 350,
     HOLDAFTER = 351,
     ISVIOLATED = 352,
     NAME = 353,
     FUNCTION_SYMBOL = 354,
     INTVAL = 355,
     FLOATVAL = 356,
     AT_TIME = 357,
     PLUS = 358,
     HYPHEN = 359,
     DIV = 360,
     MUL = 361,
     UMINUS = 362
   };
#endif
/* Tokens.  */
#define OPEN_BRAC 258
#define CLOSE_BRAC 259
#define OPEN_SQ 260
#define CLOSE_SQ 261
#define DEFINE 262
#define PDDLDOMAIN 263
#define REQS 264
#define EQUALITY 265
#define STRIPS 266
#define ADL 267
#define NEGATIVE_PRECONDITIONS 268
#define TYPING 269
#define DISJUNCTIVE_PRECONDS 270
#define EXT_PRECS 271
#define UNIV_PRECS 272
#define QUANT_PRECS 273
#define COND_EFFS 274
#define FLUENTS 275
#define TIME 276
#define DURATIVE_ACTIONS 277
#define DURATION_INEQUALITIES 278
#define CONTINUOUS_EFFECTS 279
#define DERIVED_PREDICATES 280
#define TIMED_INITIAL_LITERALS 281
#define PREFERENCES 282
#define CONSTRAINTS 283
#define ACTION_COSTS 284
#define ACTION 285
#define NUMBER 286
#define PROCESS 287
#define EVENT 288
#define DURATIVE_ACTION 289
#define DERIVED 290
#define CONSTANTS 291
#define PREDS 292
#define FUNCTIONS 293
#define TYPES 294
#define ARGS 295
#define PRE 296
#define CONDITION 297
#define PREFERENCE 298
#define START_PRE 299
#define END_PRE 300
#define EFFECTS 301
#define INITIAL_EFFECT 302
#define FINAL_EFFECT 303
#define INVARIANT 304
#define DURATION 305
#define AT_START 306
#define AT_END 307
#define OVER_ALL 308
#define AND 309
#define OR 310
#define EXISTS 311
#define FORALL 312
#define IMPLY 313
#define NOT 314
#define WHEN 315
#define EITHER 316
#define PROBLEM 317
#define FORDOMAIN 318
#define INITIALLY 319
#define OBJECTS 320
#define GOALS 321
#define EQ 322
#define LENGTH 323
#define SERIAL 324
#define PARALLEL 325
#define METRIC 326
#define MINIMIZE 327
#define MAXIMIZE 328
#define HASHT 329
#define DURATION_VAR 330
#define TOTAL_TIME 331
#define INCREASE 332
#define DECREASE 333
#define SCALE_UP 334
#define SCALE_DOWN 335
#define ASSIGN 336
#define GREATER 337
#define GREATEQ 338
#define LESS 339
#define LESSEQ 340
#define Q 341
#define COLON 342
#define ALWAYS 343
#define SOMETIME 344
#define WITHIN 345
#define ATMOSTONCE 346
#define SOMETIMEAFTER 347
#define SOMETIMEBEFORE 348
#define ALWAYSWITHIN 349
#define HOLDDURING 350
#define HOLDAFTER 351
#define ISVIOLATED 352
#define NAME 353
#define FUNCTION_SYMBOL 354
#define INTVAL 355
#define FLOATVAL 356
#define AT_TIME 357
#define PLUS 358
#define HYPHEN 359
#define DIV 360
#define MUL 361
#define UMINUS 362




/* Copy the first part of user declarations.  */
#line 21 "pddl+.yacc"


    /*Preamble for files generated by bison.

      - Because I have no use for
        the plan parser, I am going
        to remove that
        functionality.
	
     */
    
    /* Srathclyde PG says :: 
       --------------------------------------------------------
      - Error reporting:
      Intention is to provide error token on most bracket expressions,
      so synchronisation can occur on next CLOSE_BRAC.
      Hence error should be generated for innermost expression containing error.
      Expressions which cause errors return a NULL values, and parser
      always attempts to carry on.
      This won't behave so well if CLOSE_BRAC is missing.

      - Naming conventions:
      Generally, the names should be similar to the PDDL2.1 spec.
      During development, they have also been based on older PDDL specs,
      older PDDL+ and TIM parsers, and this shows in places.

      - All the names of fields in the semantic value type begin with t_
      Corresponding categories in the grammar begin with c_
      Corresponding classes have no prefix.

      PDDL grammar       yacc grammar      type of corresponding semantic val.  

-      thing+             c_things          thing_list
      (thing+)           c_thing_list      thing_list

    */

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <ctype.h>
    
    
    // This is now copied locally to avoid relying on installation 
    // of flex++.

    //#include "FlexLexer.h"
    //#include <FlexLexer.h>

    // #include "ptree.h"
    // #include "parse_error.h"

#define YYDEBUG 1 

    int yyerror(char *);


#include "global.hh"
#include "Problem.hh"    
#include "Action_templates.hh"
#include "PredicatesAndPropositions_templates.hh"

    
    extern int yylex();
    
    using namespace Planning;
    
    /*Problem associated with \global{domain} of \class{Domain}.*/
    Problem problem;

    /*Constant integer cost of an action.*/
    int totalCost = 0;

    /*Predicate that has to be evaluated to determine the cost of an action.*/
    SignedPredicate* costEvaluator = 0;
    

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

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 98 "pddl+.yacc"
{

    
    /* Simple type for storing numbers.*/
    IntAndDouble* intAndDouble;
    
    /* --------------------- Higher level components of a PDDL spec.  ------------------- */


    /*Types have type.*/
    TypeOfTypes* typeOfTypes;

    /*Objects and constants have type.*/
    TypeOfSymbols* typeOfSymbols;
    
    /*Sequence of variables.*/
    Variables* variables;

    /*Sequence of constants.*/
    Constants* constants;
    
    /*Sequence of types.*/
    Types* types;
    
    /*Specification of a symbol's arguments*/
    Arguments* arguments;

    /*A component of the specification of a symbol's arguments.*/
    ArgumentComponent* argumentComponent;

    /*Sequence of pointers to \class{Constant}s and \class{Variable}s*/
    Parameters* parameters;

    /*Sequence of pointers (each to a \class{string}).*/
    UntypedStrings* untypedStrings;

    /* --------------------- Things to do with state ---------------------------------------- */

    Proposition<>* proposition;

    SignedProposition* signedProposition;
    
    Predicate<>* predicate;
    
    SignedPredicate* signedPredicate;
    
    /* --------------------- Things to do with operators -------- --------------------------- */

    SignedPredicates* signedPredicates;
    
//     SignedPropositions* signedPropositions;
    
    /* --------------------- Various incarnations of \class{string} and \class{string}s.  --- */
    
    string* str;

    PredicateName* predicateName;
    
    Type* type;
    
    Variable* variable;

    Constant* constant;
    
    /* --------------------- CPU types.  ------------------- */
    int integer;
    
    double real;
    
    /* --------------------- FAKE IS USED FOR PDDL components we ignore.  ------------------- */
    void* fake;
}
/* Line 193 of yacc.c.  */
#line 460 "pddl+.cc"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 216 of yacc.c.  */
#line 473 "pddl+.cc"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int i)
#else
static int
YYID (i)
    int i;
#endif
{
  return i;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
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
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  6
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   837

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  108
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  117
/* YYNRULES -- Number of rules.  */
#define YYNRULES  315
/* YYNRULES -- Number of states.  */
#define YYNSTATES  716

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   362

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
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
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,    82,    83,    84,
      85,    86,    87,    88,    89,    90,    91,    92,    93,    94,
      95,    96,    97,    98,    99,   100,   101,   102,   103,   104,
     105,   106,   107
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     5,     7,    13,    18,    21,    24,    27,
      30,    33,    36,    38,    43,    48,    53,    56,    57,    60,
      62,    67,    71,    73,    75,    77,    79,    82,    83,    88,
      95,   101,   105,   107,   112,   117,   119,   120,   124,   125,
     130,   135,   137,   140,   141,   146,   151,   153,   156,   160,
     161,   163,   165,   167,   169,   174,   176,   178,   181,   182,
     185,   186,   193,   196,   199,   202,   203,   208,   211,   214,
     217,   218,   220,   222,   224,   226,   228,   233,   235,   237,
     239,   241,   243,   246,   249,   252,   255,   256,   264,   272,
     277,   282,   287,   295,   301,   303,   305,   308,   309,   314,
     319,   325,   331,   335,   340,   342,   344,   346,   348,   351,
     354,   357,   358,   364,   370,   376,   382,   388,   394,   400,
     405,   408,   409,   411,   414,   416,   418,   424,   430,   436,
     442,   447,   454,   464,   474,   476,   478,   480,   482,   485,
     486,   491,   493,   498,   500,   508,   514,   520,   526,   532,
     538,   544,   549,   555,   561,   567,   573,   575,   577,   583,
     589,   591,   593,   595,   600,   605,   607,   612,   617,   619,
     621,   623,   625,   627,   629,   631,   636,   644,   647,   652,
     658,   663,   671,   673,   676,   677,   682,   688,   690,   693,
     694,   699,   707,   712,   717,   722,   728,   733,   739,   745,
     752,   759,   765,   767,   772,   777,   782,   788,   796,   802,
     805,   806,   809,   810,   812,   814,   816,   818,   823,   828,
     833,   838,   843,   848,   853,   858,   863,   868,   873,   876,
     878,   880,   882,   884,   886,   888,   890,   896,   909,   914,
     927,   932,   945,   950,   962,   967,   971,   975,   976,   978,
     983,   986,   987,   992,   997,  1002,  1008,  1013,  1015,  1020,
    1025,  1038,  1044,  1047,  1050,  1053,  1056,  1059,  1062,  1065,
    1066,  1071,  1076,  1078,  1083,  1089,  1094,  1099,  1100,  1106,
    1108,  1110,  1114,  1116,  1118,  1120,  1125,  1129,  1133,  1137,
    1141,  1145,  1147,  1150,  1152,  1155,  1157,  1159,  1161,  1163,
    1165,  1167,  1169,  1171,  1173,  1175,  1177,  1179,  1181,  1183,
    1185,  1187,  1189,  1191,  1193,  1195
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int16 yyrhs[] =
{
     109,     0,    -1,   110,    -1,   209,    -1,     3,     7,   112,
     111,     4,    -1,     3,     7,   112,     1,    -1,   113,   111,
      -1,   208,   111,    -1,   207,   111,    -1,   190,   111,    -1,
     191,   111,    -1,   192,   111,    -1,   194,    -1,     3,     8,
      98,     4,    -1,     3,     9,   114,     4,    -1,     3,     9,
       1,     4,    -1,   114,   224,    -1,    -1,   116,   115,    -1,
     116,    -1,     3,   117,   123,     4,    -1,     3,     1,     4,
      -1,    98,    -1,    67,    -1,    98,    -1,    98,    -1,   120,
     121,    -1,    -1,     3,   122,   123,     4,    -1,     3,   122,
     123,     4,   104,    31,    -1,     3,   122,     4,   104,    31,
      -1,     3,     1,     4,    -1,    98,    -1,   124,   104,   135,
     123,    -1,   124,   104,   133,   123,    -1,   124,    -1,    -1,
      86,   129,   124,    -1,    -1,   126,   104,   135,   125,    -1,
     126,   104,   133,   125,    -1,   126,    -1,   132,   126,    -1,
      -1,   136,   104,   135,   127,    -1,   136,   104,   133,   127,
      -1,   136,    -1,   128,   131,    -1,   128,    86,   130,    -1,
      -1,    98,    -1,    98,    -1,    98,    -1,    98,    -1,     3,
      61,   137,     4,    -1,    98,    -1,    98,    -1,   136,   134,
      -1,    -1,   137,   135,    -1,    -1,   138,     3,    67,   163,
     171,     4,    -1,   138,   165,    -1,   138,   164,    -1,   138,
     139,    -1,    -1,     3,   102,   138,     4,    -1,   142,   140,
      -1,   167,   140,    -1,   166,   140,    -1,    -1,   146,    -1,
     163,    -1,   162,    -1,   167,    -1,   166,    -1,     3,    54,
     144,     4,    -1,   143,    -1,   162,    -1,   163,    -1,   145,
      -1,   168,    -1,   144,   162,    -1,   144,   163,    -1,   144,
     168,    -1,   144,   145,    -1,    -1,     3,    77,     3,   122,
       4,   100,     4,    -1,     3,    77,     3,   122,     4,   163,
       4,    -1,     3,    54,   140,     4,    -1,     3,    54,     1,
       4,    -1,     3,    54,   148,     4,    -1,     3,   185,     3,
     123,     4,   147,     4,    -1,     3,    60,   203,   147,     4,
      -1,   149,    -1,   168,    -1,   148,   147,    -1,    -1,     3,
      51,   150,     4,    -1,     3,    52,   150,     4,    -1,     3,
      77,   172,   170,     4,    -1,     3,    78,   172,   170,     4,
      -1,     3,     1,     4,    -1,     3,    54,   152,     4,    -1,
     151,    -1,   162,    -1,   163,    -1,   153,    -1,   152,   162,
      -1,   152,   163,    -1,   152,   153,    -1,    -1,     3,    81,
     172,   156,     4,    -1,     3,    77,   172,   156,     4,    -1,
       3,    78,   172,   156,     4,    -1,     3,    79,   172,   156,
       4,    -1,     3,    80,   172,   156,     4,    -1,     3,    77,
     172,   170,     4,    -1,     3,    78,   172,   170,     4,    -1,
       3,    54,   155,     4,    -1,   155,   154,    -1,    -1,   157,
      -1,    86,    75,    -1,   171,    -1,   172,    -1,     3,   103,
     156,   156,     4,    -1,     3,   104,   156,   156,     4,    -1,
       3,   106,   156,   156,     4,    -1,     3,   105,   156,   156,
       4,    -1,     3,    54,   161,     4,    -1,     3,   159,    86,
      75,   160,     4,    -1,     3,    51,     3,   159,    86,    75,
     160,     4,     4,    -1,     3,    52,     3,   159,    86,    75,
     160,     4,     4,    -1,    85,    -1,    83,    -1,    67,    -1,
     169,    -1,   161,   158,    -1,    -1,     3,    59,   187,     4,
      -1,   187,    -1,     3,    59,   189,     4,    -1,   189,    -1,
       3,   185,     3,   123,     4,   141,     4,    -1,     3,    60,
     181,   140,     4,    -1,     3,    81,   172,   169,     4,    -1,
       3,    77,   172,   169,     4,    -1,     3,    78,   172,   169,
       4,    -1,     3,    79,   172,   169,     4,    -1,     3,    80,
     172,   169,     4,    -1,     3,   104,   169,     4,    -1,     3,
     103,   169,   169,     4,    -1,     3,   104,   169,   169,     4,
      -1,     3,   106,   169,   169,     4,    -1,     3,   105,   169,
     169,     4,    -1,   171,    -1,   172,    -1,     3,   106,    74,
     169,     4,    -1,     3,   106,   169,    74,     4,    -1,    74,
      -1,   100,    -1,   101,    -1,     3,    99,   128,     4,    -1,
       3,    98,   128,     4,    -1,    99,    -1,     3,    99,   128,
       4,    -1,     3,    98,   128,     4,    -1,    99,    -1,    82,
      -1,    83,    -1,    84,    -1,    85,    -1,    67,    -1,   178,
      -1,     3,    54,   182,     4,    -1,     3,   185,     3,   123,
       4,   175,     4,    -1,     3,     4,    -1,     3,    43,   180,
       4,    -1,     3,    43,    98,   180,     4,    -1,     3,    54,
     177,     4,    -1,     3,   185,     3,   123,     4,   176,     4,
      -1,   180,    -1,   177,   176,    -1,    -1,     3,    43,   181,
       4,    -1,     3,    43,    98,   181,     4,    -1,   181,    -1,
     179,   180,    -1,    -1,     3,    54,   179,     4,    -1,     3,
     185,     3,   123,     4,   180,     4,    -1,     3,    52,   181,
       4,    -1,     3,    88,   181,     4,    -1,     3,    89,   181,
       4,    -1,     3,    90,   171,   181,     4,    -1,     3,    91,
     181,     4,    -1,     3,    92,   181,   181,     4,    -1,     3,
      93,   181,   181,     4,    -1,     3,    94,   171,   181,   181,
       4,    -1,     3,    95,   171,   171,   181,     4,    -1,     3,
      96,   171,   181,     4,    -1,   187,    -1,     3,    59,   181,
       4,    -1,     3,    54,   183,     4,    -1,     3,    55,   183,
       4,    -1,     3,    58,   181,   181,     4,    -1,     3,   184,
       3,   123,     4,   181,     4,    -1,     3,   174,   169,   169,
       4,    -1,   182,   175,    -1,    -1,   183,   181,    -1,    -1,
     185,    -1,   186,    -1,    57,    -1,    56,    -1,     3,   118,
     128,     4,    -1,     3,   118,   123,     4,    -1,     3,   119,
     128,     4,    -1,     3,    37,   115,     4,    -1,     3,    37,
       1,     4,    -1,     3,    38,   120,     4,    -1,     3,    38,
       1,     4,    -1,     3,    28,   180,     4,    -1,     3,    28,
       1,     4,    -1,     3,    28,   176,     4,    -1,     3,    28,
       1,     4,    -1,   194,   195,    -1,   195,    -1,   198,    -1,
     199,    -1,   200,    -1,   201,    -1,   197,    -1,    35,    -1,
       3,   196,   188,   181,     4,    -1,     3,    30,    98,   206,
       3,   123,     4,    41,   175,    46,   141,     4,    -1,     3,
      30,     1,     4,    -1,     3,    33,    98,   206,     3,   123,
       4,    41,   181,    46,   141,     4,    -1,     3,    33,     1,
       4,    -1,     3,    32,    98,   206,     3,   123,     4,    41,
     181,    46,   154,     4,    -1,     3,    32,     1,     4,    -1,
       3,    34,    98,   206,     3,   123,     4,    50,   158,   202,
       4,    -1,     3,    34,     1,     4,    -1,   202,    46,   147,
      -1,   202,    42,   203,    -1,    -1,   205,    -1,     3,    54,
     204,     4,    -1,   204,   203,    -1,    -1,     3,    51,   181,
       4,    -1,     3,    52,   181,     4,    -1,     3,    53,   181,
       4,    -1,     3,    43,    98,   205,     4,    -1,     3,    43,
     205,     4,    -1,    40,    -1,     3,    36,   125,     4,    -1,
       3,    39,   127,     4,    -1,     3,     7,     3,    62,    98,
       4,     3,    63,    98,     4,   210,     4,    -1,     3,     7,
       3,    62,     1,    -1,   113,   210,    -1,   211,   210,    -1,
     212,   210,    -1,   214,   210,    -1,   193,   210,    -1,   215,
     210,    -1,   216,   210,    -1,    -1,     3,    65,   125,     4,
      -1,     3,    64,   138,     4,    -1,    66,    -1,     3,   213,
     175,     4,    -1,     3,    71,   219,   220,     4,    -1,     3,
      71,     1,     4,    -1,     3,    68,   217,     4,    -1,    -1,
      69,   100,   218,    70,   100,    -1,    72,    -1,    73,    -1,
       3,   221,     4,    -1,   173,    -1,   171,    -1,    76,    -1,
       3,    97,    98,     4,    -1,     3,    76,     4,    -1,   103,
     220,   222,    -1,   104,   220,   220,    -1,   106,   220,   223,
      -1,   105,   220,   220,    -1,   220,    -1,   220,   222,    -1,
     220,    -1,   220,   223,    -1,    10,    -1,    29,    -1,    11,
      -1,    14,    -1,    13,    -1,    15,    -1,    16,    -1,    17,
      -1,    19,    -1,    20,    -1,    22,    -1,    21,    -1,    12,
      -1,    18,    -1,    23,    -1,    24,    -1,    25,    -1,    26,
      -1,    27,    -1,    28,    -1,    98,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   536,   536,   539,   548,   554,   567,   570,   575,   578,
     582,   586,   590,   596,   605,   612,   626,   627,   633,   636,
     642,   648,   661,   682,   690,   704,   714,   721,   730,   741,
     752,   767,   779,   797,   820,   842,   856,   867,   876,   886,
     905,   917,   955,   962,   974,   986,   997,  1014,  1019,  1023,
    1033,  1044,  1054,  1064,  1073,  1080,  1092,  1102,  1108,  1120,
    1127,  1135,  1161,  1167,  1175,  1182,  1192,  1203,  1212,  1217,
    1222,  1237,  1240,  1248,  1257,  1263,  1274,  1277,  1286,  1295,
    1303,  1309,  1322,  1327,  1332,  1339,  1344,  1351,  1362,  1378,
    1382,  1396,  1398,  1407,  1413,  1417,  1427,  1432,  1440,  1445,
    1450,  1457,  1464,  1475,  1478,  1487,  1491,  1495,  1506,  1509,
    1512,  1516,  1525,  1529,  1533,  1537,  1541,  1550,  1557,  1564,
    1572,  1574,  1581,  1583,  1586,  1589,  1596,  1599,  1602,  1605,
    1613,  1616,  1620,  1624,  1634,  1636,  1638,  1650,  1656,  1662,
    1671,  1682,  1695,  1707,  1719,  1729,  1740,  1745,  1751,  1756,
    1761,  1771,  1774,  1777,  1780,  1783,  1786,  1789,  1796,  1799,
    1802,  1809,  1813,  1821,  1825,  1828,  1849,  1852,  1855,  1864,
    1866,  1868,  1870,  1872,  1893,  1912,  1930,  1939,  1948,  1952,
    1956,  1960,  1969,  1977,  1983,  1992,  1998,  2004,  2021,  2027,
    2036,  2041,  2049,  2054,  2059,  2064,  2070,  2075,  2080,  2085,
    2091,  2098,  2113,  2120,  2145,  2151,  2159,  2169,  2177,  2199,
    2212,  2221,  2224,  2230,  2233,  2241,  2251,  2261,  2299,  2311,
    2337,  2339,  2351,  2356,  2369,  2374,  2389,  2394,  2411,  2414,
    2422,  2425,  2429,  2433,  2437,  2446,  2458,  2473,  2517,  2530,
    2544,  2556,  2568,  2581,  2597,  2609,  2612,  2615,  2627,  2631,
    2641,  2647,  2656,  2659,  2662,  2665,  2671,  2679,  2687,  2701,
    2712,  2725,  2734,  2738,  2741,  2750,  2759,  2763,  2769,  2773,
    2786,  2792,  2801,  2811,  2824,  2829,  2843,  2856,  2856,  2869,
    2874,  2885,  2889,  2893,  2898,  2902,  2907,  2916,  2920,  2924,
    2928,  2938,  2942,  2952,  2956,  2982,  2986,  2991,  2995,  2999,
    3003,  3007,  3011,  3015,  3019,  3023,  3027,  3033,  3046,  3051,
    3055,  3059,  3063,  3067,  3071,  3075
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "OPEN_BRAC", "CLOSE_BRAC", "OPEN_SQ",
  "CLOSE_SQ", "DEFINE", "PDDLDOMAIN", "REQS", "EQUALITY", "STRIPS", "ADL",
  "NEGATIVE_PRECONDITIONS", "TYPING", "DISJUNCTIVE_PRECONDS", "EXT_PRECS",
  "UNIV_PRECS", "QUANT_PRECS", "COND_EFFS", "FLUENTS", "TIME",
  "DURATIVE_ACTIONS", "DURATION_INEQUALITIES", "CONTINUOUS_EFFECTS",
  "DERIVED_PREDICATES", "TIMED_INITIAL_LITERALS", "PREFERENCES",
  "CONSTRAINTS", "ACTION_COSTS", "ACTION", "NUMBER", "PROCESS", "EVENT",
  "DURATIVE_ACTION", "DERIVED", "CONSTANTS", "PREDS", "FUNCTIONS", "TYPES",
  "ARGS", "PRE", "CONDITION", "PREFERENCE", "START_PRE", "END_PRE",
  "EFFECTS", "INITIAL_EFFECT", "FINAL_EFFECT", "INVARIANT", "DURATION",
  "AT_START", "AT_END", "OVER_ALL", "AND", "OR", "EXISTS", "FORALL",
  "IMPLY", "NOT", "WHEN", "EITHER", "PROBLEM", "FORDOMAIN", "INITIALLY",
  "OBJECTS", "GOALS", "EQ", "LENGTH", "SERIAL", "PARALLEL", "METRIC",
  "MINIMIZE", "MAXIMIZE", "HASHT", "DURATION_VAR", "TOTAL_TIME",
  "INCREASE", "DECREASE", "SCALE_UP", "SCALE_DOWN", "ASSIGN", "GREATER",
  "GREATEQ", "LESS", "LESSEQ", "Q", "COLON", "ALWAYS", "SOMETIME",
  "WITHIN", "ATMOSTONCE", "SOMETIMEAFTER", "SOMETIMEBEFORE",
  "ALWAYSWITHIN", "HOLDDURING", "HOLDAFTER", "ISVIOLATED", "NAME",
  "FUNCTION_SYMBOL", "INTVAL", "FLOATVAL", "AT_TIME", "PLUS", "HYPHEN",
  "DIV", "MUL", "UMINUS", "$accept", "input", "c_domain", "c_preamble",
  "c_domain_name", "c_domain_require_def", "c_reqs", "c_pred_decls",
  "c_pred_decl", "c_new_pred_symbol", "c_pred_symbol",
  "c_init_pred_symbol", "c_func_decls", "c_func_decl", "c_new_func_symbol",
  "c_typed_var_list", "c_var_symbol_list", "c_typed_consts",
  "c_new_const_symbols", "c_typed_types", "c_parameter_symbols",
  "c_declaration_var_symbol", "c_var_symbol", "c_const_symbol",
  "c_new_const_symbol", "c_either_type", "c_new_primitive_type",
  "c_primitive_type", "c_new_primitive_types", "c_primitive_types",
  "c_init_els", "c_timed_initial_literal", "c_effects", "c_effect",
  "c_a_effect", "c_p_effect", "c_p_effects", "c_action_cost_effect",
  "c_conj_effect", "c_da_effect", "c_da_effects", "c_timed_effect",
  "c_a_effect_da", "c_p_effect_da", "c_p_effects_da", "c_f_assign_da",
  "c_proc_effect", "c_proc_effects", "c_f_exp_da", "c_binary_expr_da",
  "c_duration_constraint", "c_d_op", "c_d_value", "c_duration_constraints",
  "c_neg_simple_effect", "c_pos_simple_effect", "c_init_neg_simple_effect",
  "c_init_pos_simple_effect", "c_forall_effect", "c_cond_effect",
  "c_assignment", "c_f_exp", "c_f_exp_t", "c_number", "c_f_head",
  "c_ground_f_head", "c_comparison_op", "c_pre_goal_descriptor",
  "c_pref_con_goal", "c_pref_con_goal_list", "c_pref_goal_descriptor",
  "c_constraint_goal_list", "c_constraint_goal", "c_goal_descriptor",
  "c_pre_goal_descriptor_list", "c_goal_list", "c_quantifier", "c_forall",
  "c_exists", "c_proposition", "c_derived_proposition",
  "c_init_proposition", "c_predicates", "c_functions_def",
  "c_constraints_def", "c_constraints_probdef", "c_structure_defs",
  "c_structure_def", "c_rule_head", "c_derivation_rule", "c_action_def",
  "c_event_def", "c_process_def", "c_durative_action_def", "c_da_def_body",
  "c_da_gd", "c_da_gds", "c_timed_gd", "c_args_head", "c_domain_constants",
  "c_type_names", "c_problem", "c_problem_body", "c_objects",
  "c_initial_state", "c_goals", "c_goal_spec", "c_metric_spec",
  "c_length_spec", "c_length_field", "@1", "c_optimization",
  "c_ground_f_exp", "c_binary_ground_f_exp", "c_binary_ground_f_pexps",
  "c_binary_ground_f_mexps", "c_require_key", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315,   316,   317,   318,   319,   320,   321,   322,   323,   324,
     325,   326,   327,   328,   329,   330,   331,   332,   333,   334,
     335,   336,   337,   338,   339,   340,   341,   342,   343,   344,
     345,   346,   347,   348,   349,   350,   351,   352,   353,   354,
     355,   356,   357,   358,   359,   360,   361,   362
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,   108,   109,   109,   110,   110,   111,   111,   111,   111,
     111,   111,   111,   112,   113,   113,   114,   114,   115,   115,
     116,   116,   117,   118,   118,   119,   120,   120,   121,   121,
     121,   121,   122,   123,   123,   123,   123,   124,   124,   125,
     125,   125,   126,   126,   127,   127,   127,   128,   128,   128,
     129,   130,   131,   132,   133,   134,   135,   136,   136,   137,
     137,   138,   138,   138,   138,   138,   139,   140,   140,   140,
     140,   141,   141,   141,   141,   141,   142,   142,   143,   143,
     143,   143,   144,   144,   144,   144,   144,   145,   145,   146,
     146,   147,   147,   147,   147,   147,   148,   148,   149,   149,
     149,   149,   149,   150,   150,   151,   151,   151,   152,   152,
     152,   152,   153,   153,   153,   153,   153,   154,   154,   154,
     155,   155,   156,   156,   156,   156,   157,   157,   157,   157,
     158,   158,   158,   158,   159,   159,   159,   160,   161,   161,
     162,   163,   164,   165,   166,   167,   168,   168,   168,   168,
     168,   169,   169,   169,   169,   169,   169,   169,   170,   170,
     170,   171,   171,   172,   172,   172,   173,   173,   173,   174,
     174,   174,   174,   174,   175,   175,   175,   175,   176,   176,
     176,   176,   176,   177,   177,   178,   178,   178,   179,   179,
     180,   180,   180,   180,   180,   180,   180,   180,   180,   180,
     180,   180,   181,   181,   181,   181,   181,   181,   181,   182,
     182,   183,   183,   184,   184,   185,   186,   187,   188,   189,
     190,   190,   191,   191,   192,   192,   193,   193,   194,   194,
     195,   195,   195,   195,   195,   196,   197,   198,   198,   199,
     199,   200,   200,   201,   201,   202,   202,   202,   203,   203,
     204,   204,   205,   205,   205,   205,   205,   206,   207,   208,
     209,   209,   210,   210,   210,   210,   210,   210,   210,   210,
     211,   212,   213,   214,   215,   215,   216,   218,   217,   219,
     219,   220,   220,   220,   220,   220,   220,   221,   221,   221,
     221,   222,   222,   223,   223,   224,   224,   224,   224,   224,
     224,   224,   224,   224,   224,   224,   224,   224,   224,   224,
     224,   224,   224,   224,   224,   224
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     1,     5,     4,     2,     2,     2,     2,
       2,     2,     1,     4,     4,     4,     2,     0,     2,     1,
       4,     3,     1,     1,     1,     1,     2,     0,     4,     6,
       5,     3,     1,     4,     4,     1,     0,     3,     0,     4,
       4,     1,     2,     0,     4,     4,     1,     2,     3,     0,
       1,     1,     1,     1,     4,     1,     1,     2,     0,     2,
       0,     6,     2,     2,     2,     0,     4,     2,     2,     2,
       0,     1,     1,     1,     1,     1,     4,     1,     1,     1,
       1,     1,     2,     2,     2,     2,     0,     7,     7,     4,
       4,     4,     7,     5,     1,     1,     2,     0,     4,     4,
       5,     5,     3,     4,     1,     1,     1,     1,     2,     2,
       2,     0,     5,     5,     5,     5,     5,     5,     5,     4,
       2,     0,     1,     2,     1,     1,     5,     5,     5,     5,
       4,     6,     9,     9,     1,     1,     1,     1,     2,     0,
       4,     1,     4,     1,     7,     5,     5,     5,     5,     5,
       5,     4,     5,     5,     5,     5,     1,     1,     5,     5,
       1,     1,     1,     4,     4,     1,     4,     4,     1,     1,
       1,     1,     1,     1,     1,     4,     7,     2,     4,     5,
       4,     7,     1,     2,     0,     4,     5,     1,     2,     0,
       4,     7,     4,     4,     4,     5,     4,     5,     5,     6,
       6,     5,     1,     4,     4,     4,     5,     7,     5,     2,
       0,     2,     0,     1,     1,     1,     1,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     2,     1,
       1,     1,     1,     1,     1,     1,     5,    12,     4,    12,
       4,    12,     4,    11,     4,     3,     3,     0,     1,     4,
       2,     0,     4,     4,     4,     5,     4,     1,     4,     4,
      12,     5,     2,     2,     2,     2,     2,     2,     2,     0,
       4,     4,     1,     4,     5,     4,     4,     0,     5,     1,
       1,     3,     1,     1,     1,     4,     3,     3,     3,     3,
       3,     1,     2,     1,     2,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint16 yydefact[] =
{
       0,     0,     0,     2,     3,     0,     1,     0,     0,     0,
       0,     5,     0,     0,     0,     0,     0,     0,    12,   229,
     234,   230,   231,   232,   233,     0,     0,     0,   261,     0,
       0,     0,     0,     0,     0,     0,   235,    43,     0,     0,
      58,     0,     4,     6,     9,    10,    11,     0,   228,     8,
       7,    13,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,    53,     0,    41,    43,
       0,     0,     0,    19,     0,     0,     0,    46,     0,     0,
       0,    15,    14,   295,   297,   307,   299,   298,   300,   301,
     302,   308,   303,   304,   306,   305,   309,   310,   311,   312,
     313,   314,   296,   315,    16,   225,     0,   189,   215,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,   224,
     238,   257,     0,   242,     0,   240,     0,   244,     0,   258,
       0,    42,   221,     0,    22,    36,   220,    18,   223,     0,
     222,    26,   259,    55,     0,    57,    23,    24,    36,     0,
       0,   202,     0,     0,     0,     0,     0,   161,   162,     0,
       0,     0,     0,     0,     0,     0,    36,    36,    36,    36,
      36,     0,    56,    43,    43,    21,     0,     0,    35,     0,
      32,    38,    58,    58,     0,   212,   212,   216,     0,     0,
     173,   169,   170,   171,   172,    49,     0,     0,   213,   214,
     236,     0,   192,   190,   188,   193,   194,     0,   196,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,    60,
      40,    39,    50,    38,    20,     0,    31,     0,     0,    45,
      44,   218,     0,     0,     0,     0,     0,     0,   165,     0,
     156,   157,    36,   269,   195,   197,   198,     0,     0,   201,
       0,     0,     0,     0,     0,     0,    37,    36,    36,     0,
      28,   204,   211,   205,     0,   203,   217,     0,    52,    47,
      49,    49,     0,     0,     0,     0,     0,     0,     0,   269,
     269,     0,   269,   269,   269,   269,   269,   199,   200,     0,
       0,     0,     0,     0,    54,    59,    34,    33,    30,     0,
     206,    51,    48,     0,     0,     0,     0,     0,     0,   208,
       0,     0,    65,    43,   272,     0,     0,     0,   262,   266,
     260,   263,   264,   265,   267,   268,   191,     0,     0,   174,
     187,     0,     0,     0,   247,    29,   164,   163,     0,   151,
       0,     0,     0,     0,     0,     0,     0,   182,     0,     0,
       0,     0,     0,   279,   280,     0,     0,   177,     0,   210,
       0,     0,     0,     0,     0,     0,   139,   136,   135,   134,
       0,     0,   152,   153,   155,   154,   207,   227,     0,   184,
       0,   226,     0,   271,    64,    63,    62,   143,   270,   277,
     276,   275,     0,   284,   168,   283,   282,     0,   273,     0,
       0,     0,    36,     0,     0,    71,    73,    72,    75,    74,
     141,     0,     0,     0,     0,     0,     0,     0,   243,     0,
       0,     0,     0,     0,    36,     0,     0,    25,    65,    49,
       0,     0,     0,    49,    49,     0,     0,     0,     0,     0,
     274,     0,   185,   175,   209,     0,     0,     0,     0,     0,
     237,   121,     0,     0,   241,   239,     0,     0,   130,   138,
       0,     0,   246,   248,     0,   245,    94,    95,     0,   178,
     180,   183,     0,     0,     0,     0,     0,     0,     0,     0,
     286,     0,     0,     0,     0,     0,     0,     0,   281,   186,
       0,     0,     0,     0,    70,    77,    80,    78,    79,    70,
      70,    81,     0,    70,    36,     0,     0,     0,     0,     0,
       0,     0,   137,     0,     0,     0,     0,   251,     0,     0,
       0,    97,     0,     0,     0,     0,     0,     0,     0,   179,
       0,   142,     0,    66,   219,   278,   285,   167,   166,   291,
     287,   288,   290,   293,   289,     0,    90,    86,     0,     0,
      89,    67,    69,    68,   140,     0,     0,   119,   120,     0,
     160,     0,     0,     0,     0,   131,     0,     0,     0,     0,
       0,     0,     0,   102,     0,     0,   104,   107,   105,   106,
       0,     0,     0,     0,     0,     0,     0,     0,    36,     0,
       0,    61,   292,   294,   176,     0,     0,     0,     0,   145,
       0,     0,   117,   118,     0,     0,     0,   256,   252,   253,
     254,   249,   250,   111,     0,     0,     0,     0,     0,    98,
      99,    91,    96,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,   181,     0,    76,    85,    82,    83,    84,
      49,     0,     0,     0,     0,     0,     0,   255,     0,     0,
       0,     0,     0,     0,    93,     0,   147,   100,   148,   101,
     149,   150,   146,     0,     0,   144,     0,     0,   132,   133,
       0,   103,   110,   108,   109,     0,     0,     0,   122,   124,
     125,     0,     0,     0,     0,     0,     0,     0,     0,   158,
     159,     0,     0,     0,     0,   123,   113,   114,   115,   116,
     112,    92,    87,    88,     0,     0,     0,     0,     0,     0,
       0,     0,   126,   127,   129,   128
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     2,     3,    13,     8,   279,    54,    72,    73,   135,
     195,   429,    75,   141,   181,   177,   178,    67,    68,    76,
     303,   223,   302,   269,    69,   173,   145,   174,    77,   255,
     348,   384,   493,   404,   494,   495,   595,   496,   405,   465,
     581,   466,   575,   576,   648,   577,   412,   505,   677,   678,
     334,   370,   511,   416,   497,   498,   385,   386,   499,   500,
     501,   512,   561,   240,   241,   396,   196,   328,   346,   423,
     329,   154,   347,   330,   401,   232,   197,   449,   199,   151,
      79,   387,    15,    16,    17,   280,    18,    19,    41,    20,
      21,    22,    23,    24,   371,   462,   572,   463,   122,    25,
      26,     4,   281,   282,   283,   317,   284,   285,   286,   351,
     430,   355,   539,   439,   540,   544,   104
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -484
static const yytype_int16 yypact[] =
{
      71,    81,   100,  -484,  -484,   198,  -484,   195,   224,   114,
      55,  -484,   619,   218,   223,   223,   223,   223,   234,  -484,
    -484,  -484,  -484,  -484,  -484,   223,   223,   237,  -484,   241,
     457,   245,    56,    65,    72,    78,  -484,   156,   250,    58,
    -484,   273,  -484,  -484,  -484,  -484,  -484,   430,  -484,  -484,
    -484,  -484,   278,   254,   491,   283,   437,   295,   302,    69,
     311,    69,   319,    69,   325,    69,  -484,   328,   243,   156,
     362,    79,   380,   385,   386,    66,   389,   117,   -16,   401,
     359,  -484,  -484,  -484,  -484,  -484,  -484,  -484,  -484,  -484,
    -484,  -484,  -484,  -484,  -484,  -484,  -484,  -484,  -484,  -484,
    -484,  -484,  -484,  -484,  -484,  -484,   401,  -484,  -484,   401,
     401,   168,   401,   401,   401,   168,   168,   168,   428,  -484,
    -484,  -484,   440,  -484,   442,  -484,   453,  -484,   456,  -484,
      84,  -484,  -484,   429,  -484,   -46,  -484,  -484,  -484,   106,
    -484,  -484,  -484,  -484,    84,  -484,  -484,  -484,   -46,   279,
     486,  -484,   394,   493,   314,   532,   541,  -484,  -484,   401,
     543,   401,   401,   401,   168,   401,   -46,   -46,   -46,   -46,
     -46,   499,  -484,   156,   156,  -484,   469,   570,   475,   580,
    -484,   205,  -484,  -484,   587,  -484,  -484,  -484,   401,   401,
     176,  -484,  -484,  -484,  -484,  -484,    89,   600,  -484,  -484,
    -484,   610,  -484,  -484,  -484,  -484,  -484,   611,  -484,   612,
     613,   401,   401,   614,   628,   639,   641,   642,   644,  -484,
    -484,  -484,  -484,   518,  -484,    84,  -484,   556,   646,  -484,
    -484,  -484,   317,   352,   401,   655,   177,   331,  -484,    89,
    -484,  -484,   -46,   658,  -484,  -484,  -484,   662,   664,  -484,
     674,   621,   637,   638,   630,   204,  -484,   -46,   -46,   652,
     588,  -484,  -484,  -484,   689,  -484,  -484,   596,  -484,  -484,
    -484,  -484,    89,    89,    89,    89,   697,   703,   321,   658,
     658,   704,   658,   658,   658,   658,   658,  -484,  -484,   705,
     707,   401,   401,   708,  -484,  -484,  -484,  -484,  -484,   681,
    -484,  -484,  -484,   196,   209,    89,    33,    89,    89,  -484,
     401,   308,  -484,   156,  -484,   645,   232,   707,  -484,  -484,
    -484,  -484,  -484,  -484,  -484,  -484,  -484,   285,   667,  -484,
    -484,   669,   670,   483,  -484,  -484,  -484,  -484,   713,  -484,
     714,   715,   716,   717,   718,   545,   719,  -484,   356,   720,
     625,   722,   723,  -484,  -484,    49,   724,  -484,   112,  -484,
     726,   727,   728,   727,   729,   730,  -484,  -484,  -484,  -484,
     648,    26,  -484,  -484,  -484,  -484,  -484,  -484,   119,  -484,
     732,  -484,   255,  -484,  -484,  -484,  -484,  -484,  -484,  -484,
    -484,  -484,   566,  -484,  -484,  -484,  -484,   733,  -484,   401,
     734,   368,   -46,   494,   735,  -484,  -484,  -484,  -484,  -484,
    -484,   157,   736,   737,   -20,   -20,   375,   661,  -484,   739,
     740,   674,   741,   378,   -46,   743,   744,  -484,  -484,  -484,
     678,   745,   653,  -484,  -484,    49,    49,    49,    49,   746,
    -484,   748,  -484,  -484,  -484,   749,   235,   744,   401,   751,
    -484,  -484,    11,    11,  -484,  -484,   671,   672,  -484,  -484,
      89,   398,  -484,  -484,   492,  -484,  -484,  -484,   752,  -484,
    -484,  -484,   755,   657,   756,   -16,   168,   407,   210,   663,
    -484,   757,   212,   215,    49,    49,    49,    49,  -484,  -484,
     707,   758,   346,   760,   762,  -484,  -484,  -484,  -484,   762,
     762,  -484,   763,   762,   -46,   411,   303,   203,   203,   691,
     693,   765,  -484,   120,   401,   401,   401,  -484,   766,   768,
     768,  -484,   739,    11,    11,    11,    11,    11,   769,  -484,
     770,  -484,   771,  -484,  -484,  -484,  -484,  -484,  -484,    49,
    -484,  -484,  -484,    49,  -484,   772,  -484,  -484,    46,    11,
    -484,  -484,  -484,  -484,  -484,   773,   774,  -484,  -484,   668,
    -484,   775,   776,    89,    89,  -484,    24,   778,   779,   780,
     781,   782,   436,  -484,   531,   783,  -484,  -484,  -484,  -484,
     784,   484,   740,    40,    40,    89,    89,    89,   -46,   785,
     705,  -484,  -484,  -484,  -484,   495,   458,    89,    89,  -484,
     727,    57,  -484,  -484,   786,   787,   788,  -484,  -484,  -484,
    -484,  -484,  -484,  -484,    11,    11,    11,    11,    11,  -484,
    -484,  -484,  -484,   789,   583,   790,   791,   792,   793,   794,
     795,   796,   797,  -484,   340,  -484,  -484,  -484,  -484,  -484,
     798,   799,   800,    89,   731,   802,   803,  -484,   555,    61,
      61,    61,    61,    61,  -484,    57,  -484,  -484,  -484,  -484,
    -484,  -484,  -484,   740,   105,  -484,   804,   805,  -484,  -484,
     546,  -484,  -484,  -484,  -484,   592,   738,   806,  -484,  -484,
    -484,   807,   808,   810,   811,    68,   812,   813,   814,  -484,
    -484,    61,    61,    61,    61,  -484,  -484,  -484,  -484,  -484,
    -484,  -484,  -484,  -484,    61,    61,    61,    61,   815,   816,
     817,   818,  -484,  -484,  -484,  -484
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -484,  -484,  -484,   605,  -484,   659,  -484,   709,  -484,  -484,
     666,  -484,  -484,  -484,   227,  -146,   601,  -140,   759,   404,
    -141,  -484,  -484,  -484,  -484,  -135,  -484,   -96,  -484,  -484,
     397,  -484,  -149,  -360,  -484,  -484,  -484,   231,  -484,  -383,
    -484,  -484,   307,  -484,  -484,   181,   326,  -484,  -129,  -484,
     414,   186,    43,  -484,  -336,  -345,  -484,  -484,  -351,  -335,
    -403,  -189,  -317,  -111,  -421,  -484,  -484,  -288,  -404,  -484,
    -484,  -484,   -30,   -68,  -484,   647,  -484,   -48,  -484,  -348,
    -484,   409,  -484,  -484,  -484,  -484,  -484,   819,  -484,  -484,
    -484,  -484,  -484,  -484,  -484,  -483,  -484,  -306,   392,  -484,
    -484,  -484,   420,  -484,  -484,  -484,  -484,  -484,  -484,  -484,
    -484,  -484,  -300,  -484,   293,   292,  -484
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -71
static const yytype_int16 yytable[] =
{
     159,    57,   184,   413,   163,   164,   165,   239,   118,   182,
     408,   150,   408,   410,   506,   410,   407,   467,   407,   471,
     214,   215,   216,   217,   218,   406,   409,   406,   409,   356,
     418,   507,   508,   220,   221,   228,   237,   339,   153,   582,
     176,   155,   156,   624,   160,   161,   162,   367,   183,   596,
     276,   146,   392,   212,   236,   397,    28,    58,   -38,    74,
     237,   -27,   -27,   368,   675,   369,    60,   513,   419,   139,
     140,   237,   420,    62,     1,   514,   515,   516,   410,    64,
     133,   476,   147,   305,   306,   307,   308,   171,     5,   612,
     257,   207,   237,   209,   210,   211,   277,   213,   410,   502,
       6,   198,   583,   584,   585,   586,   587,   179,   475,   121,
     238,   296,   297,   444,   560,   149,   338,   340,   341,   342,
     234,   235,    56,   566,   204,   393,   589,   597,   598,   258,
     304,   643,   238,   157,   158,   484,   485,   486,   487,   238,
     157,   158,   667,   247,   248,   238,   410,   676,   394,   157,
     158,   410,   410,    29,    59,   410,   238,   157,   158,   295,
     238,   157,   158,    61,   262,   262,   264,   238,   157,   158,
      63,   410,   410,   349,   579,   579,    65,   134,   467,   467,
     -23,   266,   172,   578,   578,   541,   542,   543,   238,   157,
     158,   562,   639,   649,   650,   651,   652,   653,   622,   623,
     336,     7,   545,     9,   180,   687,   559,   568,   294,   227,
     399,   451,    27,   337,   534,   143,   537,   421,   567,   538,
     289,   144,    42,   331,   332,    11,    12,    12,   680,   680,
     680,   680,   680,   352,   452,   453,   491,    47,   492,   -70,
     642,    51,   343,   543,   395,    52,    55,   410,    56,   408,
     638,    70,   410,    71,    66,   407,   445,    10,    81,   637,
     467,   606,   -23,   267,   406,   409,   626,   628,   157,   158,
     680,   680,   680,   680,   -23,   268,    78,   560,   472,   360,
     686,    80,   267,   680,   680,   680,   680,   105,   478,   357,
     400,   176,   482,   483,   268,   267,   267,   380,   267,   119,
     410,   267,   172,   674,   353,   354,   120,   268,   268,   344,
     268,   345,   673,   268,   425,   123,   410,    56,   203,   688,
     149,   261,   426,   125,   395,   395,   395,   395,   358,   127,
      30,   441,   129,   185,   186,   187,   108,   188,   189,   359,
     186,   187,   108,   188,   189,   551,   190,   130,   422,   311,
     552,   553,   190,   427,   555,   149,   263,   428,   556,   382,
     383,   191,   192,   193,   194,   532,   132,   191,   192,   193,
     194,   327,   443,   395,   395,   395,   395,   147,   333,   458,
     503,   345,   470,   147,   136,   312,   313,   314,    71,   315,
     138,   468,   316,   142,   625,   627,   629,   630,   631,   447,
     547,   270,   271,   108,   149,   447,   448,   146,   625,   627,
     382,   533,   644,   146,   411,   557,   528,   548,   549,   525,
     526,   527,   152,   548,   549,   525,   526,   527,   395,   270,
     271,   166,   395,   175,   272,   273,   274,   275,   147,   461,
     611,   513,   632,   167,   147,   168,   569,   570,   571,   514,
     515,   516,   517,   124,   666,   126,   169,   128,    53,   170,
      32,   -17,    33,    34,    35,    36,   685,   -17,   -17,   -17,
     -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,   -17,
     -17,   -17,   -17,   -17,   -17,   -17,   -17,   464,   621,   106,
     200,   107,   201,   518,   108,    82,   342,   202,   634,   635,
     590,    83,    84,    85,    86,    87,    88,    89,    90,    91,
      92,    93,    94,    95,    96,    97,    98,    99,   100,   101,
     102,   681,   682,   683,   684,   109,   110,   111,   112,   113,
     114,   115,   116,   117,   364,   365,   205,   366,   679,   679,
     679,   679,   679,   519,   520,   206,   521,   208,   446,   108,
     367,   108,   522,   447,   448,   -17,   640,   271,   670,   671,
     219,   146,   704,   705,   706,   707,   368,   222,   369,   523,
     524,   525,   526,   527,   224,   708,   709,   710,   711,   225,
     679,   679,   679,   679,   226,   613,   229,   230,   378,   103,
     447,   231,   147,   679,   679,   679,   679,   106,   146,   379,
     456,   457,   108,   242,   176,   447,   604,   605,   614,   615,
     616,   617,   618,   146,   243,   244,   245,   246,   249,    43,
      44,    45,    46,   614,   615,   616,   617,   618,    30,   147,
      49,    50,   250,   109,   110,   111,   112,   113,   114,   115,
     116,   117,   431,   251,   147,   252,   253,    31,   254,    32,
     260,    33,    34,    35,    36,    37,    38,    39,    40,   265,
     259,   278,   290,   432,   433,   434,   287,    14,   288,   435,
     436,   437,   438,    14,    14,    14,    14,    56,   291,   292,
     293,   270,   271,   298,    14,    14,   272,   273,   274,   655,
     270,   271,   299,   300,   301,   691,   692,   693,   694,   318,
     319,   309,   321,   322,   323,   324,   325,   310,   320,   326,
     327,   333,   335,   361,   350,   362,   363,   372,   373,   374,
     375,   376,   377,   381,   388,   389,   390,   391,   398,   402,
     403,   411,   414,   415,   417,   424,   460,   440,   442,   450,
     454,   455,   461,   464,   148,   469,   473,   475,   479,   480,
     488,   481,   489,   490,   504,   427,   529,   509,   510,   530,
     531,   536,   546,   535,   550,   492,   563,   554,   564,   565,
     573,   574,   588,   345,   601,   591,   594,   599,   600,   602,
     603,   566,   137,   607,   608,   609,   610,   619,   620,   633,
     645,   646,   647,   654,   656,   657,   658,   659,   660,   661,
     662,   663,   -32,   664,   665,   667,   668,   669,   689,   690,
     696,   697,   698,   695,   699,   700,   701,   702,   703,   712,
     713,   714,   715,   641,   256,   477,   636,   580,   131,   672,
     459,   558,   592,   233,   474,   593,     0,    48
};

static const yytype_int16 yycheck[] =
{
     111,    31,   148,   363,   115,   116,   117,   196,    56,   144,
     361,    79,   363,   361,     3,   363,   361,   420,   363,   423,
     166,   167,   168,   169,   170,   361,   361,   363,   363,   317,
       4,   452,   453,   173,   174,   181,     3,     4,   106,   522,
      86,   109,   110,     3,   112,   113,   114,    67,   144,     3,
     239,    67,     3,   164,   195,   355,     1,     1,   104,     1,
       3,     3,     4,    83,     3,    85,     1,    43,    42,     3,
       4,     3,    46,     1,     3,    51,    52,    53,   426,     1,
       1,   426,    98,   272,   273,   274,   275,     3,     7,   572,
     225,   159,     3,   161,   162,   163,   242,   165,   446,   447,
       0,   149,   523,   524,   525,   526,   527,     1,     3,    40,
      99,   257,   258,   401,    74,     3,   305,   306,   307,   308,
     188,   189,     3,     3,   154,    76,   530,   548,   549,   225,
     271,    74,    99,   100,   101,   435,   436,   437,   438,    99,
     100,   101,    74,   211,   212,    99,   494,    86,    99,   100,
     101,   499,   500,    98,    98,   503,    99,   100,   101,   255,
      99,   100,   101,    98,   232,   233,   234,    99,   100,   101,
      98,   519,   520,   313,   519,   520,    98,    98,   581,   582,
       4,     4,    98,   519,   520,   485,   486,   487,    99,   100,
     101,   508,   595,   614,   615,   616,   617,   618,   581,   582,
       4,     3,   490,     8,    98,   100,     3,   513,     4,     4,
      98,    54,    98,     4,     4,    98,     4,    98,    98,     4,
     250,   104,     4,   291,   292,     1,     3,     3,   649,   650,
     651,   652,   653,     1,    77,    78,     1,     3,     3,     4,
     600,     4,   310,   543,   355,     4,     1,   595,     3,   600,
     595,     1,   600,     3,    98,   600,   402,    62,     4,   595,
     663,   567,    86,    86,   600,   600,   583,   584,   100,   101,
     691,   692,   693,   694,    98,    98,     3,    74,   424,   327,
     663,     3,    86,   704,   705,   706,   707,     4,   429,     4,
     358,    86,   433,   434,    98,    86,    86,   345,    86,     4,
     648,    86,    98,   648,    72,    73,     4,    98,    98,     1,
      98,     3,   648,    98,    59,     4,   664,     3,     4,   664,
       3,     4,    67,     4,   435,   436,   437,   438,    43,     4,
       9,   399,     4,    54,    55,    56,    57,    58,    59,    54,
      55,    56,    57,    58,    59,   494,    67,   104,   378,    28,
     499,   500,    67,    98,   503,     3,     4,   102,   504,     3,
       4,    82,    83,    84,    85,   476,     4,    82,    83,    84,
      85,     3,     4,   484,   485,   486,   487,    98,     3,     4,
     448,     3,     4,    98,     4,    64,    65,    66,     3,    68,
       4,   421,    71,     4,   583,   584,   585,   586,   587,    59,
      54,    98,    99,    57,     3,    59,    60,    67,   597,   598,
       3,     4,   601,    67,     3,     4,   464,    77,    78,    79,
      80,    81,    63,    77,    78,    79,    80,    81,   539,    98,
      99,     3,   543,     4,   103,   104,   105,   106,    98,     3,
       4,    43,   588,     3,    98,     3,   514,   515,   516,    51,
      52,    53,    54,    61,   643,    63,     3,    65,     1,     3,
      30,     4,    32,    33,    34,    35,   655,    10,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    20,    21,    22,
      23,    24,    25,    26,    27,    28,    29,     3,     4,    52,
       4,    54,    98,     1,    57,     4,   685,     4,     3,     4,
     530,    10,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    20,    21,    22,    23,    24,    25,    26,    27,    28,
      29,   650,   651,   652,   653,    88,    89,    90,    91,    92,
      93,    94,    95,    96,    51,    52,     4,    54,   649,   650,
     651,   652,   653,    51,    52,     4,    54,     4,    54,    57,
      67,    57,    60,    59,    60,    98,    98,    99,     3,     4,
      61,    67,   691,   692,   693,   694,    83,    98,    85,    77,
      78,    79,    80,    81,     4,   704,   705,   706,   707,   104,
     691,   692,   693,   694,     4,    54,   182,   183,    43,    98,
      59,     4,    98,   704,   705,   706,   707,    52,    67,    54,
     414,   415,    57,     3,    86,    59,   563,   564,    77,    78,
      79,    80,    81,    67,     4,     4,     4,     4,     4,    14,
      15,    16,    17,    77,    78,    79,    80,    81,     9,    98,
      25,    26,     4,    88,    89,    90,    91,    92,    93,    94,
      95,    96,    76,     4,    98,     4,     4,    28,     4,    30,
       4,    32,    33,    34,    35,    36,    37,    38,    39,     4,
     104,     3,    41,    97,    98,    99,     4,     8,     4,   103,
     104,   105,   106,    14,    15,    16,    17,     3,    41,    41,
      50,    98,    99,    31,    25,    26,   103,   104,   105,   106,
      98,    99,   104,     4,    98,   103,   104,   105,   106,   279,
     280,     4,   282,   283,   284,   285,   286,     4,     4,     4,
       3,     3,    31,    46,    69,    46,    46,     4,     4,     4,
       4,     4,     4,     4,     4,   100,     4,     4,     4,     3,
       3,     3,     3,     3,    86,     3,    75,     4,     4,     4,
       4,     4,     3,     3,    78,     4,     3,     3,    70,     4,
       4,    98,     4,     4,     3,    98,     4,    86,    86,     4,
       4,     4,     4,   100,     4,     3,    75,     4,    75,     4,
       4,     3,     3,     3,   106,     4,     4,     4,     4,     4,
       4,     3,    73,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,    74,     4,     4,     4,     4,
       4,     4,     4,    75,     4,     4,     4,     4,     4,     4,
       4,     4,     4,   596,   223,   428,   595,   520,    69,   648,
     416,   505,   539,   186,   425,   543,    -1,    18
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,   109,   110,   209,     7,     0,     3,   112,     8,
      62,     1,     3,   111,   113,   190,   191,   192,   194,   195,
     197,   198,   199,   200,   201,   207,   208,    98,     1,    98,
       9,    28,    30,    32,    33,    34,    35,    36,    37,    38,
      39,   196,     4,   111,   111,   111,   111,     3,   195,   111,
     111,     4,     4,     1,   114,     1,     3,   180,     1,    98,
       1,    98,     1,    98,     1,    98,    98,   125,   126,   132,
       1,     3,   115,   116,     1,   120,   127,   136,     3,   188,
       3,     4,     4,    10,    11,    12,    13,    14,    15,    16,
      17,    18,    19,    20,    21,    22,    23,    24,    25,    26,
      27,    28,    29,    98,   224,     4,    52,    54,    57,    88,
      89,    90,    91,    92,    93,    94,    95,    96,   185,     4,
       4,    40,   206,     4,   206,     4,   206,     4,   206,     4,
     104,   126,     4,     1,    98,   117,     4,   115,     4,     3,
       4,   121,     4,    98,   104,   134,    67,    98,   118,     3,
     181,   187,    63,   181,   179,   181,   181,   100,   101,   171,
     181,   181,   181,   171,   171,   171,     3,     3,     3,     3,
       3,     3,    98,   133,   135,     4,    86,   123,   124,     1,
      98,   122,   133,   135,   123,    54,    55,    56,    58,    59,
      67,    82,    83,    84,    85,   118,   174,   184,   185,   186,
       4,    98,     4,     4,   180,     4,     4,   181,     4,   181,
     181,   181,   171,   181,   123,   123,   123,   123,   123,    61,
     125,   125,    98,   129,     4,   104,     4,     4,   123,   127,
     127,     4,   183,   183,   181,   181,   128,     3,    99,   169,
     171,   172,     3,     4,     4,     4,     4,   181,   181,     4,
       4,     4,     4,     4,     4,   137,   124,   133,   135,   104,
       4,     4,   181,     4,   181,     4,     4,    86,    98,   131,
      98,    99,   103,   104,   105,   106,   169,   123,     3,   113,
     193,   210,   211,   212,   214,   215,   216,     4,     4,   180,
      41,    41,    41,    50,     4,   135,   123,   123,    31,   104,
       4,    98,   130,   128,   128,   169,   169,   169,   169,     4,
       4,    28,    64,    65,    66,    68,    71,   213,   210,   210,
       4,   210,   210,   210,   210,   210,     4,     3,   175,   178,
     181,   181,   181,     3,   158,    31,     4,     4,   169,     4,
     169,   169,   169,   181,     1,     3,   176,   180,   138,   125,
      69,   217,     1,    72,    73,   219,   175,     4,    43,    54,
     185,    46,    46,    46,    51,    52,    54,    67,    83,    85,
     159,   202,     4,     4,     4,     4,     4,     4,    43,    54,
     185,     4,     3,     4,   139,   164,   165,   189,     4,   100,
       4,     4,     3,    76,    99,   171,   173,   220,     4,    98,
     181,   182,     3,     3,   141,   146,   162,   163,   166,   167,
     187,     3,   154,   141,     3,     3,   161,    86,     4,    42,
      46,    98,   180,   177,     3,    59,    67,    98,   102,   119,
     218,    76,    97,    98,    99,   103,   104,   105,   106,   221,
       4,   181,     4,     4,   175,   123,    54,    59,    60,   185,
       4,    54,    77,    78,     4,     4,   159,   159,     4,   158,
      75,     3,   203,   205,     3,   147,   149,   168,   180,     4,
       4,   176,   123,     3,   189,     3,   163,   138,   128,    70,
       4,    98,   128,   128,   220,   220,   220,   220,     4,     4,
       4,     1,     3,   140,   142,   143,   145,   162,   163,   166,
     167,   168,   187,   181,     3,   155,     3,   172,   172,    86,
      86,   160,   169,    43,    51,    52,    53,    54,     1,    51,
      52,    54,    60,    77,    78,    79,    80,    81,   185,     4,
       4,     4,   171,     4,     4,   100,     4,     4,     4,   220,
     222,   220,   220,   220,   223,   175,     4,    54,    77,    78,
       4,   140,   140,   140,     4,   140,   123,     4,   154,     3,
      74,   170,   170,    75,    75,     4,     3,    98,   205,   181,
     181,   181,   204,     4,     3,   150,   151,   153,   162,   163,
     150,   148,   203,   172,   172,   172,   172,   172,     3,   176,
     180,     4,   222,   223,     4,   144,     3,   172,   172,     4,
       4,   106,     4,     4,   160,   160,   205,     4,     4,     4,
       4,     4,   203,    54,    77,    78,    79,    80,    81,     4,
       4,     4,   147,   147,     3,   169,   170,   169,   170,   169,
     169,   169,   123,     4,     3,     4,   145,   162,   163,   168,
      98,   122,   141,    74,   169,     4,     4,     4,   152,   172,
     172,   172,   172,   172,     4,   106,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,   169,    74,     4,     4,
       3,     4,   153,   162,   163,     3,    86,   156,   157,   171,
     172,   156,   156,   156,   156,   169,   147,   100,   163,     4,
       4,   103,   104,   105,   106,    75,     4,     4,     4,     4,
       4,     4,     4,     4,   156,   156,   156,   156,   156,   156,
     156,   156,     4,     4,     4,     4
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


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
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
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
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *bottom, yytype_int16 *top)
#else
static void
yy_stack_print (bottom, top)
    yytype_int16 *bottom;
    yytype_int16 *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      fprintf (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      fprintf (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
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
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The look-ahead symbol.  */
int yychar;

/* The semantic value of the look-ahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  int yystate;
  int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Look-ahead token as an internal (translated) token number.  */
  int yytoken = 0;
#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  yytype_int16 yyssa[YYINITDEPTH];
  yytype_int16 *yyss = yyssa;
  yytype_int16 *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  YYSTYPE *yyvsp;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

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
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;


	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),

		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
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

  /* Do appropriate processing given the current state.  Read a
     look-ahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to look-ahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a look-ahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid look-ahead symbol.  */
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
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
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

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the look-ahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

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
#line 536 "pddl+.yacc"
    {
    VERBOSE("Parsed domain definition.");
;}
    break;

  case 3:
#line 539 "pddl+.yacc"
    {
    VERBOSE("Parsed problem.");
    ;}
    break;

  case 4:
#line 549 "pddl+.yacc"
    {
//     $$= $4;
//     $$->name= $3;
//     delete [] $3;
;}
    break;

  case 5:
#line 555 "pddl+.yacc"
    {
    yyerrok;
    //$$=static_cast<domain*>(NULL);

    UNRECOVERABLE_ERROR("Syntax error in domain specification.");
    
    //log_error(E_FATAL,"Syntax error in domain");
;}
    break;

  case 6:
#line 567 "pddl+.yacc"
    {
    //$$= $2; $$->req= $1;
;}
    break;

  case 7:
#line 570 "pddl+.yacc"
    {

    problem.domain.setTypes((yyvsp[(1) - (2)].typeOfTypes));
    
;}
    break;

  case 8:
#line 575 "pddl+.yacc"
    {
    problem.domain.setConstants((yyvsp[(1) - (2)].typeOfSymbols));
;}
    break;

  case 9:
#line 578 "pddl+.yacc"
    {
    //$$= $2; 
    //$$->predicates= $1;
;}
    break;

  case 10:
#line 582 "pddl+.yacc"
    {
    //$$= $2; 
    //$$->functions= $1;
;}
    break;

  case 11:
#line 586 "pddl+.yacc"
    {
    //$$= $2;
    //$$->constraints = $1;
;}
    break;

  case 12:
#line 590 "pddl+.yacc"
    {
    //$$= new SuperType($1);
  ;}
    break;

  case 13:
#line 596 "pddl+.yacc"
    {
    (yyval.str) = (yyvsp[(3) - (4)].str);
;}
    break;

  case 14:
#line 606 "pddl+.yacc"
    {
/*     // Stash in analysis object --- we need to refer to it during parse */
/*     //   but domain object is not created yet, */
/*     current_analysis->req |= $3; */
/*     $$=$3; */
;}
    break;

  case 15:
#line 613 "pddl+.yacc"
    {
    yyerrok; 
    UNRECOVERABLE_ERROR("Syntax error in requirements declaration.");
/*     log_error(E_FATAL,"Syntax error in requirements declaration."); */
/*     $$= 0; */
;}
    break;

  case 16:
#line 626 "pddl+.yacc"
    { ;}
    break;

  case 17:
#line 627 "pddl+.yacc"
    { ;}
    break;

  case 18:
#line 634 "pddl+.yacc"
    {
;}
    break;

  case 19:
#line 637 "pddl+.yacc"
    {
;}
    break;

  case 20:
#line 643 "pddl+.yacc"
    {
    problem.domain.predicateSpecifications[*(yyvsp[(2) - (4)].predicateName)] = *(yyvsp[(3) - (4)].arguments);
    FAKE_DELETE((yyvsp[(3) - (4)].arguments));
    FAKE_DELETE((yyvsp[(2) - (4)].predicateName));
;}
    break;

  case 21:
#line 649 "pddl+.yacc"
    {
    yyerrok;
    UNRECOVERABLE_ERROR("Syntax error in predicate declaration.");
    // hope someone makes this error someday
/*     log_error(E_FATAL,"Syntax error in predicate declaration."); */
/*     $$= NULL; */
;}
    break;

  case 22:
#line 662 "pddl+.yacc"
    {
    VERBOSER(1, "New predicate symbol"<<NAME<<endl);

    (yyval.predicateName) = new PredicateName(*(yyvsp[(1) - (1)].str));
    
    problem.domain.predicates.insert(*(yyval.predicateName));
    
    FAKE_DELETE((yyvsp[(1) - (1)].str));
    
/*     $$=current_analysis->pred_tab.symbol_put($1); */
/*     current_analysis->var_tab_stack.push( */
/* 	current_analysis->buildPredTab()); */
/*     delete [] $1; */
;}
    break;

  case 23:
#line 682 "pddl+.yacc"
    {
    //$$=current_analysis->pred_tab.symbol_ref("=");

    (yyval.predicateName) = new PredicateName("=");
    
    problem.domain.requires.equality = true;
    //requires(E_EQUALITY);
;}
    break;

  case 24:
#line 690 "pddl+.yacc"
    {

    (yyval.predicateName) = new PredicateName(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
    ;}
    break;

  case 25:
#line 704 "pddl+.yacc"
    {
    (yyval.predicateName) = new PredicateName(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
    //$$=current_analysis->pred_tab.symbol_get($1); delete [] $1;
;}
    break;

  case 26:
#line 715 "pddl+.yacc"
    {
    WARNING("We do not support functor (c_func_decls) declarations.");

    //$$ = NULL;//new SuperType($2);
    
;}
    break;

  case 27:
#line 721 "pddl+.yacc"
    {
    WARNING("We do not support functor (c_func_decls) declarations.");
    //$$ = NULL;//new SuperType();
;}
    break;

  case 28:
#line 731 "pddl+.yacc"
    {
    WARNING("We do not support functor (c_func_decl) declarations."<<endl
	    <<"The symbol involved is :: "<<*(yyvsp[(2) - (4)].str)<<" and a typed variable list :: "<<endl
	    <<*(yyvsp[(3) - (4)].arguments)<<endl);
    
    FAKE_DELETE((yyvsp[(2) - (4)].str));
    
    //$$ = NULL;//new SuperType();//($2, $3);
    //$$= new func_decl($2,$3,current_analysis->var_tab_stack.pop());
;}
    break;

  case 29:
#line 742 "pddl+.yacc"
    {
    WARNING("We do not support functor (c_func_decl) declarations."<<endl
	    <<"The symbol involved is :: "<<*(yyvsp[(2) - (6)].str)<<" and a typed variable list :: "<<endl
	    <<*(yyvsp[(3) - (6)].arguments)<<endl);
    
    FAKE_DELETE((yyvsp[(2) - (6)].str));
    
    //$$ = NULL;//new SuperType();//($2, $3);
    //$$= new func_decl($2,$3,current_analysis->var_tab_stack.pop());
;}
    break;

  case 30:
#line 753 "pddl+.yacc"
    {
    string* functionName = (yyvsp[(2) - (5)].str);
    VERBOSER(19, "Adding new function with name :: "<<*functionName<<endl);

    assert(*functionName == string("total-cost"));

    problem.domain.addFunction<uint>(*functionName, Parameters());
    
    //$$ = new PositiverIntegerFunction(*functionName, Parameters());
    
    //$$ = new Function(*functionName, );

    FAKE_DELETE(functionName);
;}
    break;

  case 31:
#line 768 "pddl+.yacc"
    {
    //$$ = NULL;
    yyerrok;
    WARNING("Syntax error in functor (c_func_decl) declaration.");
/*     log_error(E_FATAL,"Syntax error in functor declaration."); */
/*     $$= NULL; */ ;}
    break;

  case 32:
#line 780 "pddl+.yacc"
    {
    (yyval.str) = (yyvsp[(1) - (1)].str);
    WARNING("We do not support functor (c_new_func_symbol) declarations."<<endl
			<<"The symbol involved is :: "<<*(yyvsp[(1) - (1)].str)<<"."<<endl);

    
//     $$ = new String($1);
    
//     domain.functionSymbols.insert(string($1));
//     FAKE_DELETE($1);
;}
    break;

  case 33:
#line 798 "pddl+.yacc"
    {
    
    /*Our objective is to concatenate the "typed variable list"
     * \union{arguments} \at{c_typed_var_list} with a new
     * list of variables \union{variables}
     * \at{c_var_symbol_list} which have the type \union{variable}
     * \at{c_primitive_type}.*/

    /*We must have already passed some of a typed variables list
     * \at{c_typed_var_list }*/
    (yyval.arguments) = (yyvsp[(4) - (4)].arguments);

    Types tmp;
    tmp.push_back(*(yyvsp[(3) - (4)].type));//Type($3));
    ArgumentComponent argumentComponent(tmp, *(yyvsp[(1) - (4)].variables));//Variables($1->begin(), $1->end()));
    (yyval.arguments)->push_back(argumentComponent);
    FAKE_DELETE((yyvsp[(1) - (4)].variables));
    FAKE_DELETE((yyvsp[(3) - (4)].type));
    
    /*At this point we know that the domain requires typed variables.*/
    problem.domain.requires.typing = true;
;}
    break;

  case 34:
#line 821 "pddl+.yacc"
    {
    
    
    /*Our objective is to concatenate the "typed variable list"
     * \union{arguments} \at{c_typed_var_list} with a new
     * list of variables \union{variables}
     * \at{c_var_symbol_list} which have of or more of the types in
     * \union{types} \at{c_primitive_type}.*/
    
    
    (yyval.arguments) = (yyvsp[(4) - (4)].arguments);

    ArgumentComponent argumentComponent(*(yyvsp[(3) - (4)].types), *(yyvsp[(1) - (4)].variables));//Variables($1->begin(), $1->end()));
    (yyval.arguments)->push_back(argumentComponent);
    FAKE_DELETE((yyvsp[(1) - (4)].variables));
    FAKE_DELETE((yyvsp[(3) - (4)].types));

    
    /*At this point we know that the domain requires typed variables.*/
    problem.domain.requires.typing = true;
;}
    break;

  case 35:
#line 843 "pddl+.yacc"
    {
    /*A, possibly empty, list of untyped variables.*/
    (yyval.arguments) = new Arguments;

    if((yyvsp[(1) - (1)].variables)->size() > 0){
	ArgumentComponent argumentComponent(Types(), *(yyvsp[(1) - (1)].variables));
	(yyval.arguments)->push_back(argumentComponent);
    }
    
    
    FAKE_DELETE((yyvsp[(1) - (1)].variables));
;}
    break;

  case 36:
#line 856 "pddl+.yacc"
    {
    (yyval.arguments) = new Arguments;
;}
    break;

  case 37:
#line 868 "pddl+.yacc"
    {
    /*Add the \union{variable} \at{c_declaration_var_symbol} to the
     * \union{variables} \at{c_var_symbol_list}*/
    
    (yyval.variables)=(yyvsp[(3) - (3)].variables);
    (yyvsp[(3) - (3)].variables)->push_back(*(yyvsp[(2) - (3)].variable));
    FAKE_DELETE((yyvsp[(2) - (3)].variable));
;}
    break;

  case 38:
#line 876 "pddl+.yacc"
    {
    (yyval.variables) = new Variables();//var_symbol_list;
;}
    break;

  case 39:
#line 886 "pddl+.yacc"
    {

    /*We have that \union{untypedStrings} \at{c_new_const_symbols} are
     * untyped \class{string}s that should cast to either
     * \class{Constant} or \class{Type}. \at{c_primitive_type} we have
     * a \union{type}.*/
    
    (yyval.typeOfSymbols) = (yyvsp[(4) - (4)].typeOfSymbols);
    
    Types tmp;
    tmp.push_back(*(yyvsp[(3) - (4)].type));
    TypeOfSymbol typeOfSymbol(tmp, *(yyvsp[(1) - (4)].untypedStrings));//Variables($1->begin(), $1->end()));
    (yyval.typeOfSymbols)->push_back(typeOfSymbol);
    FAKE_DELETE((yyvsp[(1) - (4)].untypedStrings));
    FAKE_DELETE((yyvsp[(3) - (4)].type));
    
    problem.domain.requires.typing = true;
    
;}
    break;

  case 40:
#line 906 "pddl+.yacc"
    {
    (yyval.typeOfSymbols) = (yyvsp[(4) - (4)].typeOfSymbols);
    
    TypeOfSymbol typeOfSymbol(*(yyvsp[(3) - (4)].types), *(yyvsp[(1) - (4)].untypedStrings));//Variables($1->begin(), $1->end()));
    (yyval.typeOfSymbols)->push_back(typeOfSymbol);
    FAKE_DELETE((yyvsp[(1) - (4)].untypedStrings));
    FAKE_DELETE((yyvsp[(3) - (4)].types));
    
    problem.domain.requires.typing = true;
;}
    break;

  case 41:
#line 917 "pddl+.yacc"
    {
    /*A, possibly empty, list of untyped type-or-constant symbols.*/
    (yyval.typeOfSymbols) = new TypeOfSymbols;

    if((yyvsp[(1) - (1)].untypedStrings)->size() > 0){
	TypeOfSymbol typeOfSymbol(Types(), *(yyvsp[(1) - (1)].untypedStrings));
	(yyval.typeOfSymbols)->push_back(typeOfSymbol);
    }
    
    FAKE_DELETE((yyvsp[(1) - (1)].untypedStrings));
;}
    break;

  case 42:
#line 955 "pddl+.yacc"
    {
    (yyval.untypedStrings) = (yyvsp[(2) - (2)].untypedStrings);
    (yyval.untypedStrings)->push_back((yyvsp[(1) - (2)].constant));
    //problem.constants.insert(*$1);
    //FAKE_DELETE($1);
//     $$=$2; $2->push_front($1);
;}
    break;

  case 43:
#line 962 "pddl+.yacc"
    {
    (yyval.untypedStrings)= new UntypedStrings();
;}
    break;

  case 44:
#line 975 "pddl+.yacc"
    {
    (yyval.typeOfTypes) = (yyvsp[(4) - (4)].typeOfTypes);
    
    Types tmp;
    tmp.push_back(*(yyvsp[(3) - (4)].type));
    TypeOfType typeOfType(tmp, *(yyvsp[(1) - (4)].types));
    (yyval.typeOfTypes)->push_back(typeOfType);
    
    FAKE_DELETE((yyvsp[(1) - (4)].types));
    FAKE_DELETE((yyvsp[(3) - (4)].type));
;}
    break;

  case 45:
#line 987 "pddl+.yacc"
    {
    (yyval.typeOfTypes) = (yyvsp[(4) - (4)].typeOfTypes);

    TypeOfType typeOfType(*(yyvsp[(3) - (4)].types), *(yyvsp[(1) - (4)].types));
    (yyval.typeOfTypes)->push_back(typeOfType);
    
    FAKE_DELETE((yyvsp[(1) - (4)].types));
    FAKE_DELETE((yyvsp[(3) - (4)].types));
;}
    break;

  case 46:
#line 998 "pddl+.yacc"
    {
    (yyval.typeOfTypes) = new TypeOfTypes();

    if((yyvsp[(1) - (1)].types)->size() > 0){
	TypeOfType typeOfType(Types(), *(yyvsp[(1) - (1)].types));
	(yyval.typeOfTypes)->push_back(typeOfType);
    }
    
    FAKE_DELETE((yyvsp[(1) - (1)].types));
;}
    break;

  case 47:
#line 1015 "pddl+.yacc"
    {
    (yyval.parameters)=(yyvsp[(1) - (2)].parameters);
    (yyval.parameters)->push_back((yyvsp[(2) - (2)].constant));
;}
    break;

  case 48:
#line 1020 "pddl+.yacc"
    {
    (yyval.parameters)=(yyvsp[(1) - (3)].parameters);
    (yyval.parameters)->push_back((yyvsp[(3) - (3)].variable)); ;}
    break;

  case 49:
#line 1023 "pddl+.yacc"
    {
    (yyval.parameters)= new Parameters();
    ;}
    break;

  case 50:
#line 1033 "pddl+.yacc"
    {
    (yyval.variable) = new Variable(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 51:
#line 1044 "pddl+.yacc"
    { 
    (yyval.variable) = new Variable(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 52:
#line 1054 "pddl+.yacc"
    { 
    (yyval.constant) = new Constant(*(yyvsp[(1) - (1)].str));
    //problem.constants.insert(*$$);
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 53:
#line 1064 "pddl+.yacc"
    {
    (yyval.constant) = new Constant(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 54:
#line 1074 "pddl+.yacc"
    { (yyval.types) = (yyvsp[(3) - (4)].types); ;}
    break;

  case 55:
#line 1081 "pddl+.yacc"
    {
    (yyval.type) = new Type(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 56:
#line 1093 "pddl+.yacc"
    { 
    (yyval.type) = new Type(*(yyvsp[(1) - (1)].str));
    FAKE_DELETE((yyvsp[(1) - (1)].str));
;}
    break;

  case 57:
#line 1103 "pddl+.yacc"
    {
    (yyval.types)= (yyvsp[(1) - (2)].types);
    (yyval.types)->push_back(*(yyvsp[(2) - (2)].type));
    FAKE_DELETE((yyvsp[(2) - (2)].type));
;}
    break;

  case 58:
#line 1108 "pddl+.yacc"
    {

    
    (yyval.types) = new Types();
    
;}
    break;

  case 59:
#line 1121 "pddl+.yacc"
    {
    (yyval.types) = (yyvsp[(1) - (2)].types);
    (yyval.types)->push_back(*(yyvsp[(2) - (2)].type));
    FAKE_DELETE((yyvsp[(2) - (2)].type));
    //$$= $1; $$->push_back($2);
;}
    break;

  case 60:
#line 1127 "pddl+.yacc"
    {
    (yyval.types) = new Types();//new pddl_type_list;
;}
    break;

  case 61:
#line 1136 "pddl+.yacc"
    {
    VERBOSER(25, "Function occurs in starting state description (c_init_els).\n");

    SignedPredicate* groundFunctionSymbol = (yyvsp[(4) - (6)].signedPredicate);
    
    assert(groundFunctionSymbol->isPositive());
    
    /*At this point we assert that all the arguments to
     * \local{groundFunctionSymbol} are of type \type{Constant} (see
     * \module{PredicatesAndPropositions}). */
    Proposition<> proposition(*groundFunctionSymbol);
    
    problem.startingStateFunctionEvaluation[proposition]
	= (yyvsp[(5) - (6)].intAndDouble)->getVal(0);
    
    /*c_number*/
    FAKE_DELETE((yyvsp[(5) - (6)].intAndDouble));

    /*c_pos_simple_effect*/
    FAKE_DELETE((yyvsp[(4) - (6)].signedPredicate));
    
//     $$=$1;
//     $$->assign_effects.push_back(new assignment($4,E_ASSIGN,$5));  
//     requires(E_FLUENTS); 
;}
    break;

  case 62:
#line 1162 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing a initialisation add effect (c_init_els).\n");

    (yyval.signedPredicates)->push_back((yyvsp[(2) - (2)].signedProposition));
;}
    break;

  case 63:
#line 1168 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing a initialisation delete effect (c_init_els).\n");

    (yyval.signedPredicates)->push_back((yyvsp[(2) - (2)].signedProposition));
    
    WARNING("Delete effect in starting state :: "<<*(yyvsp[(2) - (2)].signedProposition));
;}
    break;

  case 64:
#line 1176 "pddl+.yacc"
    {
    WARNING("We do not deal with timed literals (c_init_els).\n");
    
    // $$=$1; $$->timed_effects.push_back($2);
;}
    break;

  case 65:
#line 1182 "pddl+.yacc"
    {
    VERBOSER(1, "Starting to parse the starting state (c_init_els).\n");

    (yyval.signedPredicates) = new SignedPredicates();
;}
    break;

  case 66:
#line 1193 "pddl+.yacc"
    {
    WARNING("We do not deal with timed literals (c_timed_initial_literal).\n");
//     requires(E_TIMED_INITIAL_LITERALS); 
//     $$=new timed_initial_literal($3,$2);
;}
    break;

  case 67:
#line 1204 "pddl+.yacc"
    {
    /*\at{c_a_effect} is a conjunction of effects.*/
    
    (yyval.signedPredicates) = (yyvsp[(2) - (2)].signedPredicates); //$$->append_effects($1); delete $1;

    (yyval.signedPredicates)->insert((yyval.signedPredicates)->begin(), (yyvsp[(1) - (2)].signedPredicates)->begin(), (yyvsp[(1) - (2)].signedPredicates)->end());
    FAKE_DELETE((yyvsp[(1) - (2)].signedPredicates));
;}
    break;

  case 68:
#line 1212 "pddl+.yacc"
    {
    
    (yyval.signedPredicates) = 0;
    WARNING("We do not support conditional effects (c_effects).");
;}
    break;

  case 69:
#line 1217 "pddl+.yacc"
    {
    
    (yyval.signedPredicates) =0;
    WARNING("We do not support universal quantification (c_effects).");
;}
    break;

  case 70:
#line 1222 "pddl+.yacc"
    {
    //$$=new effect_lists();
    VERBOSER(1, "Make a new effects list.");
    (yyval.signedPredicates) = new SignedPredicates();//AddAndDeleteList();
    ;}
    break;

  case 71:
#line 1237 "pddl+.yacc"
    {
    (yyval.signedPredicates) = (yyvsp[(1) - (1)].signedPredicates);
;}
    break;

  case 72:
#line 1240 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing simple negative effect (c_effect)."<<endl);
    (yyval.signedPredicates) = new SignedPredicates(1);//AddAndDeleteList;
    (*(yyval.signedPredicates))[0] = (yyvsp[(1) - (1)].signedPredicate);//->push_back($1);
    //FAKE_DELETE($1);
//     $$=new effect_lists;
//     $$->add_effects.push_front($1);
    ;}
    break;

  case 73:
#line 1248 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing simple negative effect (c_effect)."<<endl);
    
    (yyval.signedPredicates) = new SignedPredicates(1);//AddAndDeleteList;
    (*(yyval.signedPredicates))[0] = (yyvsp[(1) - (1)].signedPredicate);//->second.push_back(*$1);
    //FAKE_DELETE($1);
//     $$=new effect_lists;
//     $$->del_effects.push_front($1);
    ;}
    break;

  case 74:
#line 1257 "pddl+.yacc"
    {
    (yyval.signedPredicates) = NULL;
    
    WARNING("We do not deal with conditional effects (c_effect)."<<endl);
    //$$=new effect_lists; $$->cond_effects.push_front($1);
    ;}
    break;

  case 75:
#line 1263 "pddl+.yacc"
    {
    (yyval.signedPredicates) = NULL;
    
    WARNING("We do not deal with universally quantified effects (c_effect)."<<endl);
    //$$=new effect_lists; $$->forall_effects.push_front($1);
    ;}
    break;

  case 76:
#line 1274 "pddl+.yacc"
    {
    (yyval.signedPredicates)= (yyvsp[(3) - (4)].signedPredicates);
;}
    break;

  case 77:
#line 1277 "pddl+.yacc"
    {
    VERBOSER(19, "Parsing (c_p_effect) from (c_a_effect)."<<endl);
    (yyval.signedPredicates)= (yyvsp[(1) - (1)].signedPredicates);
    ;}
    break;

  case 78:
#line 1287 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing simple negative effect (c_p_effect).");
    (yyval.signedPredicates) = new SignedPredicates(1);//addAndDeleteList;
    (*(yyval.signedPredicates))[0] = (yyvsp[(1) - (1)].signedPredicate);//->second.push_back(*$1);
    //FAKE_DELETE($1);
    
    //$$=new effect_lists; $$->del_effects.push_front($1);
;}
    break;

  case 79:
#line 1296 "pddl+.yacc"
    {
    VERBOSER(1, "Parsing simple positive effect (c_p_effect).");
    (yyval.signedPredicates) = new SignedPredicates(1);//addAndDeleteList;
    (*(yyval.signedPredicates))[0] = (yyvsp[(1) - (1)].signedPredicate);//->first.push_back(*$1);
    //FAKE_DELETE($1);
    //$$=new effect_lists; $$->add_effects.push_front($1);
;}
    break;

  case 80:
#line 1304 "pddl+.yacc"
    {
    VERBOSER(19, "Parsing action cost effect."<<endl);
    (yyval.signedPredicates) = new SignedPredicates();
    //$$ = NULL;
;}
    break;

  case 81:
#line 1310 "pddl+.yacc"
    {
    (yyval.signedPredicates) = NULL;
    WARNING("We do not support assignments (c_p_effect).");
//     $$=new effect_lists; $$->assign_effects.push_front($1);
//     requires(E_FLUENTS);
;}
    break;

  case 82:
#line 1322 "pddl+.yacc"
    {
    (yyval.signedPredicates)= (yyvsp[(1) - (2)].signedPredicates);
    (yyval.signedPredicates)->push_back((yyvsp[(2) - (2)].signedPredicate));
    //FAKE_DELETE($2);
;}
    break;

  case 83:
#line 1327 "pddl+.yacc"
    {
    (yyval.signedPredicates)= (yyvsp[(1) - (2)].signedPredicates);
    (yyval.signedPredicates)->push_back((yyvsp[(2) - (2)].signedPredicate));
    //FAKE_DELETE($2);
;}
    break;

  case 84:
#line 1332 "pddl+.yacc"
    {

    (yyval.signedPredicates) = NULL;
    WARNING("We do not support assignments (c_p_effects).");
//     $$= $1; $$->assign_effects.push_back($2);
//     requires(E_FLUENTS);
;}
    break;

  case 85:
#line 1340 "pddl+.yacc"
    {
    (yyval.signedPredicates) = (yyvsp[(1) - (2)].signedPredicates);/*HERE*/
    
;}
    break;

  case 86:
#line 1344 "pddl+.yacc"
    {
    (yyval.signedPredicates)= new SignedPredicates;//addAndDeleteList();//new effect_lists;
    ;}
    break;

  case 87:
#line 1352 "pddl+.yacc"
    {
    /*HERE*/
    
    string* functionName = (yyvsp[(4) - (7)].str);

    totalCost = (yyvsp[(6) - (7)].integer);
    
    assert(*functionName == string("total-cost"));
    FAKE_DELETE(functionName);
;}
    break;

  case 88:
#line 1363 "pddl+.yacc"
    {
    /*HERE*/
    
    string* functionName = (yyvsp[(4) - (7)].str);
    
    costEvaluator = (yyvsp[(6) - (7)].signedPredicate);
    
    assert(*functionName == string("total-cost"));
    FAKE_DELETE(functionName);
;}
    break;

  case 89:
#line 1379 "pddl+.yacc"
    {
    (yyval.signedPredicates)=(yyvsp[(3) - (4)].signedPredicates);
;}
    break;

  case 90:
#line 1383 "pddl+.yacc"
    {
    yyerrok;
    (yyval.signedPredicates)=NULL;
    UNRECOVERABLE_ERROR("Broken conjunctive effect (c_conj_effect).");
    
    //log_error(E_FATAL,"Syntax error in (and ...)");
;}
    break;

  case 91:
#line 1397 "pddl+.yacc"
    { ;}
    break;

  case 92:
#line 1402 "pddl+.yacc"
    { // $$= new effect_lists; 
//     $$->forall_effects.push_back(
// 	new forall_effect($6, $4, current_analysis->var_tab_stack.pop())); 
//     requires(E_COND_EFFS);
;}
    break;

  case 93:
#line 1408 "pddl+.yacc"
    { // $$= new effect_lists;
//     $$->cond_effects.push_back(
// 	new cond_effect($3,$4));
//     requires(E_COND_EFFS);
;}
    break;

  case 94:
#line 1414 "pddl+.yacc"
    { // $$=new effect_lists;
//     $$->timed_effects.push_back($1);
;}
    break;

  case 95:
#line 1418 "pddl+.yacc"
    { // $$= new effect_lists;
//     $$->assign_effects.push_front($1);
//     requires(E_FLUENTS);
;}
    break;

  case 96:
#line 1427 "pddl+.yacc"
    {

    
    //$$=$1; $1->append_effects($2); delete $2;
;}
    break;

  case 97:
#line 1432 "pddl+.yacc"
    {
    //$$= new effect_lists;
;}
    break;

  case 98:
#line 1441 "pddl+.yacc"
    {
    WARNING("We do not support timed effects (c_timed_effect).");
    //$$=new timed_effect($3,E_AT_START);
;}
    break;

  case 99:
#line 1446 "pddl+.yacc"
    {
    WARNING("We do not support timed effects (c_timed_effect).");
    //$$=new timed_effect($3,E_AT_END);
;}
    break;

  case 100:
#line 1451 "pddl+.yacc"
    {
    WARNING("We do not support timed effects (c_timed_effect).");
//     $$=new timed_effect(new effect_lists,E_CONTINUOUS);
//     $$->effs->assign_effects.push_front(
// 	new assignment($3,E_INCREASE,$4));
;}
    break;

  case 101:
#line 1458 "pddl+.yacc"
    {
    WARNING("We do not support timed effects (c_timed_effect).");
//     $$=new timed_effect(new effect_lists,E_CONTINUOUS);
//     $$->effs->assign_effects.push_front(
// 	new assignment($3,E_DECREASE,$4));
;}
    break;

  case 102:
#line 1465 "pddl+.yacc"
    {
    yyerrok;
    WARNING("We do not support timed effects (c_timed_effect).");
//     log_error(E_FATAL,"Syntax error in timed effect");
;}
    break;

  case 103:
#line 1475 "pddl+.yacc"
    {
    //$$= $3;
;}
    break;

  case 104:
#line 1478 "pddl+.yacc"
    {
    //$$= $1;
    ;}
    break;

  case 105:
#line 1488 "pddl+.yacc"
    {
    //$$=new effect_lists; $$->del_effects.push_front($1);
;}
    break;

  case 106:
#line 1492 "pddl+.yacc"
    {
    //$$=new effect_lists; $$->add_effects.push_front($1);
;}
    break;

  case 107:
#line 1496 "pddl+.yacc"
    {
//     $$=new effect_lists; $$->assign_effects.push_front($1);
//     requires(E_FLUENTS);
;}
    break;

  case 108:
#line 1506 "pddl+.yacc"
    {
    //$$= $1; $$->del_effects.push_back($2);
;}
    break;

  case 109:
#line 1509 "pddl+.yacc"
    {
    //$$= $1; $$->add_effects.push_back($2);
;}
    break;

  case 110:
#line 1512 "pddl+.yacc"
    {
    //$$= $1; $$->assign_effects.push_back($2);
    //requires(E_FLUENTS);
;}
    break;

  case 111:
#line 1516 "pddl+.yacc"
    {
    //$$= new effect_lists;
    ;}
    break;

  case 112:
#line 1526 "pddl+.yacc"
    {
    //$$= new assignment($3,E_ASSIGN,$4);
;}
    break;

  case 113:
#line 1530 "pddl+.yacc"
    {
    //$$= new assignment($3,E_INCREASE,$4);
;}
    break;

  case 114:
#line 1534 "pddl+.yacc"
    {
    //$$= new assignment($3,E_DECREASE,$4);
;}
    break;

  case 115:
#line 1538 "pddl+.yacc"
    {
    //$$= new assignment($3,E_SCALE_UP,$4);
;}
    break;

  case 116:
#line 1542 "pddl+.yacc"
    {
    //$$= new assignment($3,E_SCALE_DOWN,$4);
;}
    break;

  case 117:
#line 1551 "pddl+.yacc"
    {// $$=new effect_lists; 
//     timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
//     $$->timed_effects.push_front(te);
//     te->effs->assign_effects.push_front(
// 	new assignment($3,E_INCREASE,$4));
;}
    break;

  case 118:
#line 1558 "pddl+.yacc"
    {// $$=new effect_lists; 
//     timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
//     $$->timed_effects.push_front(te);
//     te->effs->assign_effects.push_front(
// 	new assignment($3,E_DECREASE,$4));
;}
    break;

  case 119:
#line 1565 "pddl+.yacc"
    {// $$ = $3;
;}
    break;

  case 120:
#line 1572 "pddl+.yacc"
    { // $$=$1; $1->append_effects($2); delete $2;
;}
    break;

  case 121:
#line 1574 "pddl+.yacc"
    { // $$= new effect_lists;
;}
    break;

  case 122:
#line 1581 "pddl+.yacc"
    {// $$= $1;
;}
    break;

  case 123:
#line 1583 "pddl+.yacc"
    {// $$= new special_val_expr(E_DURATION_VAR);
//     requires( E_DURATION_INEQUALITIES );
;}
    break;

  case 124:
#line 1586 "pddl+.yacc"
    { // $$=$1;
    FAKE_DELETE((yyvsp[(1) - (1)].intAndDouble));
    ;}
    break;

  case 125:
#line 1589 "pddl+.yacc"
    { // $$= $1;
    ;}
    break;

  case 126:
#line 1597 "pddl+.yacc"
    { // $$= new plus_expression($3,$4);
;}
    break;

  case 127:
#line 1600 "pddl+.yacc"
    { // $$= new minus_expression($3,$4);
;}
    break;

  case 128:
#line 1603 "pddl+.yacc"
    { // $$= new mul_expression($3,$4);
;}
    break;

  case 129:
#line 1606 "pddl+.yacc"
    { // $$= new div_expression($3,$4);
;}
    break;

  case 130:
#line 1614 "pddl+.yacc"
    { // $$= new conj_goal($3);
;}
    break;

  case 131:
#line 1617 "pddl+.yacc"
    { // $$= new timed_goal(new comparison($2,
// 				    new special_val_expr(E_DURATION_VAR),$5),E_AT_START);
;}
    break;

  case 132:
#line 1621 "pddl+.yacc"
    { // $$ = new timed_goal(new comparison($4,
// 				     new special_val_expr(E_DURATION_VAR),$7),E_AT_START);
;}
    break;

  case 133:
#line 1625 "pddl+.yacc"
    { // $$ = new timed_goal(new comparison($4,
// 				     new special_val_expr(E_DURATION_VAR),$7),E_AT_END);
;}
    break;

  case 134:
#line 1634 "pddl+.yacc"
    {// $$= E_LESSEQ; requires(E_DURATION_INEQUALITIES);
;}
    break;

  case 135:
#line 1636 "pddl+.yacc"
    {// $$= E_GREATEQ; requires(E_DURATION_INEQUALITIES);
    ;}
    break;

  case 136:
#line 1638 "pddl+.yacc"
    {// $$= E_EQUALS;
    ;}
    break;

  case 137:
#line 1650 "pddl+.yacc"
    {// $$= $1;
;}
    break;

  case 138:
#line 1657 "pddl+.yacc"
    {
    WARNING("We do not support durational constraints.\n");
    //$$=$1; $$->push_back($2);
;}
    break;

  case 139:
#line 1662 "pddl+.yacc"
    {
    WARNING("We do not support durational constraints.\n");
    //$$= new goal_list;
;}
    break;

  case 140:
#line 1672 "pddl+.yacc"
    {
    (yyval.signedPredicate) = new SignedPredicate(*(yyvsp[(3) - (4)].signedPredicate));
    (yyval.signedPredicate)->makeNegative();
    FAKE_DELETE((yyvsp[(3) - (4)].signedPredicate));
;}
    break;

  case 141:
#line 1683 "pddl+.yacc"
    {
    (yyval.signedPredicate) = new SignedPredicate(*(yyvsp[(1) - (1)].signedPredicate));
    (yyval.signedPredicate)->makePositive();
    FAKE_DELETE((yyvsp[(1) - (1)].signedPredicate));
;}
    break;

  case 142:
#line 1696 "pddl+.yacc"
    {
    assert((yyvsp[(3) - (4)].signedProposition));
    (yyval.signedProposition) = (yyvsp[(3) - (4)].signedProposition);
    (yyval.signedProposition)->makeNegative();
    
;}
    break;

  case 143:
#line 1708 "pddl+.yacc"
    {
    assert((yyval.signedProposition));
    (yyval.signedProposition) = (yyvsp[(1) - (1)].signedProposition);
    (yyval.signedProposition)->makePositive();
    
;}
    break;

  case 144:
#line 1720 "pddl+.yacc"
    { 
    WARNING("We do not deal with universally quantified effects (c_forall_effect).");
    //$$= new forall_effect($6, $4, current_analysis->var_tab_stack.pop());
;}
    break;

  case 145:
#line 1730 "pddl+.yacc"
    {
    WARNING("We do not deal with conditional effects (c_cond_effect).");
    //$$= new cond_effect($3,$4);
;}
    break;

  case 146:
#line 1741 "pddl+.yacc"
    {
    WARNING("We do not support assignments (c_assignment).");
    //$$= new assignment($3,E_ASSIGN,$4);
;}
    break;

  case 147:
#line 1746 "pddl+.yacc"
    {
    
    WARNING("We do not support assignments (c_assignment).");
    //$$= new assignment($3,E_INCREASE,$4);
;}
    break;

  case 148:
#line 1752 "pddl+.yacc"
    {
    WARNING("We do not support assignments (c_assignment).");
    //$$= new assignment($3,E_DECREASE,$4);
;}
    break;

  case 149:
#line 1757 "pddl+.yacc"
    {
    WARNING("We do not support assignments (c_assignment).");
    //$$= new assignment($3,E_SCALE_UP,$4);
;}
    break;

  case 150:
#line 1762 "pddl+.yacc"
    {
    WARNING("We do not support assignments (c_assignment).");
    //$$= new assignment($3,E_SCALE_DOWN,$4);
;}
    break;

  case 151:
#line 1772 "pddl+.yacc"
    { // $$= new uminus_expression($3); requires(E_FLUENTS);
;}
    break;

  case 152:
#line 1775 "pddl+.yacc"
    { // $$= new plus_expression($3,$4); requires(E_FLUENTS);
;}
    break;

  case 153:
#line 1778 "pddl+.yacc"
    { // $$= new minus_expression($3,$4); requires(E_FLUENTS);
;}
    break;

  case 154:
#line 1781 "pddl+.yacc"
    { // $$= new mul_expression($3,$4); requires(E_FLUENTS);
;}
    break;

  case 155:
#line 1784 "pddl+.yacc"
    { // $$= new div_expression($3,$4); requires(E_FLUENTS);
;}
    break;

  case 156:
#line 1786 "pddl+.yacc"
    { // $$=$1;
    FAKE_DELETE((yyvsp[(1) - (1)].intAndDouble));
    ;}
    break;

  case 157:
#line 1789 "pddl+.yacc"
    { // $$= $1; requires(E_FLUENTS);
    ;}
    break;

  case 158:
#line 1797 "pddl+.yacc"
    { // $$= new mul_expression(new special_val_expr(E_HASHT),$4);
;}
    break;

  case 159:
#line 1800 "pddl+.yacc"
    { // $$= new mul_expression($3, new special_val_expr(E_HASHT));
;}
    break;

  case 160:
#line 1803 "pddl+.yacc"
    { // $$= new special_val_expr(E_HASHT);
;}
    break;

  case 161:
#line 1809 "pddl+.yacc"
    {
    (yyval.intAndDouble) = new IntAndDouble((yyvsp[(1) - (1)].integer));
    // $$=new int_expression($1);
;}
    break;

  case 162:
#line 1813 "pddl+.yacc"
    { 
    (yyval.intAndDouble) = new IntAndDouble((yyvsp[(1) - (1)].real));
    // $$=new float_expression($1);
  ;}
    break;

  case 163:
#line 1822 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($2), $3); delete [] $2;
;}
    break;

  case 164:
#line 1826 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($2), $3); delete [] $2;
;}
    break;

  case 165:
#line 1829 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($1),
// 		    new parameter_symbol_list); delete [] $1;
;}
    break;

  case 166:
#line 1850 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($2), $3); delete [] $2;
;}
    break;

  case 167:
#line 1853 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($2), $3); delete [] $2;
;}
    break;

  case 168:
#line 1856 "pddl+.yacc"
    { // $$=new func_term( current_analysis->func_tab.symbol_get($1),
// 		    new parameter_symbol_list); delete [] $1;
;}
    break;

  case 169:
#line 1864 "pddl+.yacc"
    { // $$= E_GREATER;
;}
    break;

  case 170:
#line 1866 "pddl+.yacc"
    { // $$= E_GREATEQ;
   ;}
    break;

  case 171:
#line 1868 "pddl+.yacc"
    { // $$= E_LESS;
   ;}
    break;

  case 172:
#line 1870 "pddl+.yacc"
    { // $$= E_LESSEQ;
   ;}
    break;

  case 173:
#line 1872 "pddl+.yacc"
    { // $$= E_EQUALS;
   ;}
    break;

  case 174:
#line 1894 "pddl+.yacc"
    {
    assert((yyvsp[(1) - (1)].signedPredicates));
    
    (yyval.signedPredicates) = (yyvsp[(1) - (1)].signedPredicates);
 //    /*Make sure that our application supports whatever type of was
//      * parsed (i.e. null test).*/
//     if($1 != 0){
// // 	$$ = new SignedPredicates();
// // 	$$->push_back($1);
// // 	FAKE_DELETE($1);
// 	//$$ = $1;
//     }
    
    VERBOSER(1, "We do not accommodate preferences in goals... However any goal can be\n"
	     <<"parsed as a preference goal.\n");
    
    //$$= $1;
;}
    break;

  case 175:
#line 1913 "pddl+.yacc"
    {
    /*As far as I can tell, we must parse a conjunctive goal in the
      propositional planning case.*/
    
    VERBOSER(1, "Parse propositional goal -- Might also be a precondition."<<endl);

    assert((yyvsp[(3) - (4)].signedPredicates));
    
    /*Because we have no choice but to parse a conjunctive goal, I
     * figure we may as-well parse that at the lower level, knowing
     * full well it is going to be a conjunction.*/
    (yyval.signedPredicates) = (yyvsp[(3) - (4)].signedPredicates);
    
    //$$ = $3;
    
    //$$ = new conj_goal($3);
;}
    break;

  case 176:
#line 1932 "pddl+.yacc"
    {
    (yyval.signedPredicates) = 0;
    WARNING("We do not support universal quantification.\n");
    
    //$$= new qfied_goal(E_FORALL,$4,$6,current_analysis->var_tab_stack.pop());
    //requires(E_UNIV_PRECS);
;}
    break;

  case 177:
#line 1940 "pddl+.yacc"
    {
    (yyval.signedPredicates) = new SignedPredicates();
;}
    break;

  case 178:
#line 1949 "pddl+.yacc"
    {
    //$$ = new preference($3);requires(E_PREFERENCES);
;}
    break;

  case 179:
#line 1953 "pddl+.yacc"
    {
    //$$ = new preference($3,$4);requires(E_PREFERENCES);
;}
    break;

  case 180:
#line 1957 "pddl+.yacc"
    {
    //$$ = new conj_goal($3);
;}
    break;

  case 181:
#line 1962 "pddl+.yacc"
    {
    //$$= new qfied_goal(E_FORALL,$4,$6,current_analysis->var_tab_stack.pop());
    //requires(E_UNIV_PRECS);

    VERBOSER(1, "Domain requires UNIVERSAL-PRECONDITIONS (c_pref_con_goal).");  
    problem.domain.requires.universallyQuantifiedPreconditions = true;
;}
    break;

  case 182:
#line 1970 "pddl+.yacc"
    {
    //$$ = $1;
;}
    break;

  case 183:
#line 1978 "pddl+.yacc"
    {
    VERBOSER(1, "We don't deal with preferences (c_pref_con_goal_list).\n");
    //$$=$1; $1->push_back($2);
;}
    break;

  case 184:
#line 1983 "pddl+.yacc"
    {
    VERBOSER(1, "We don't deal with preferences (c_pref_con_goal_list).\n");
    //$$= new goal_list;
;}
    break;

  case 185:
#line 1993 "pddl+.yacc"
    {
    WARNING("We don't deal with preferences (c_pref_goal_descriptor).\n");
    //$$= new preference($3); requires(E_PREFERENCES);
    (yyval.signedPredicates) = NULL;
;}
    break;

  case 186:
#line 1999 "pddl+.yacc"
    {
    WARNING("We don't deal with preferences (c_pref_goal_descriptor).\n");
    //$$= new preference($3,$4); requires(E_PREFERENCES);
    (yyval.signedPredicates) = NULL;
;}
    break;

  case 187:
#line 2005 "pddl+.yacc"
    {
    VERBOSER(1, "Using rule for goal with preferences (\rule{c_pref_goal_descriptor})"<<endl
	    <<"to parse ordinary \rule{c_goal_descriptor}.");
    //VERBOSE("We don't deal with preferences (c_pref_goal_descriptor).\n");

    assert((yyvsp[(1) - (1)].signedPredicates));
    
    (yyval.signedPredicates) = (yyvsp[(1) - (1)].signedPredicates);
    
    //$$=$1;
;}
    break;

  case 188:
#line 2022 "pddl+.yacc"
    {
    VERBOSE("We don't deal with constraints as part of the goal (c_pref_goal_descriptor).\n");
    //$$ = $1; $$->push_back($2);
;}
    break;

  case 189:
#line 2027 "pddl+.yacc"
    {
    VERBOSE("We don't deal with constraints as part of the goal (c_pref_goal_descriptor).\n");
    //$$ = new goal_list;
;}
    break;

  case 190:
#line 2037 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$= new conj_goal($3);
;}
    break;

  case 191:
#line 2042 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    VERBOSER(1, "Domain requires UNIVERSAL-PRECONDITIONS (c_constraint_goal).");  
    problem.domain.requires.universallyQuantifiedPreconditions;
    // $$ = new qfied_goal(E_FORALL,$4,$6,current_analysis->var_tab_stack.pop());
    // requires(E_UNIV_PRECS);
;}
    break;

  case 192:
#line 2050 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_ATEND,$3);
;}
    break;

  case 193:
#line 2055 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_ALWAYS,$3);
;}
    break;

  case 194:
#line 2060 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_SOMETIME,$3);
;}
    break;

  case 195:
#line 2065 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    FAKE_DELETE((yyvsp[(3) - (5)].intAndDouble));
    // $$ = new constraint_goal(E_WITHIN,$4,NULL,$3->double_value(),0.0);delete $3;
;}
    break;

  case 196:
#line 2071 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_ATMOSTONCE,$3);
;}
    break;

  case 197:
#line 2076 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_SOMETIMEAFTER,$4,$3);
;}
    break;

  case 198:
#line 2081 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    // $$ = new constraint_goal(E_SOMETIMEBEFORE,$4,$3);
;}
    break;

  case 199:
#line 2086 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    FAKE_DELETE((yyvsp[(3) - (6)].intAndDouble));
    // $$ = new constraint_goal(E_ALWAYSWITHIN,$5,$4,$3->double_value(),0.0);delete $3;
;}
    break;

  case 200:
#line 2092 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    FAKE_DELETE((yyvsp[(3) - (6)].intAndDouble));
    FAKE_DELETE((yyvsp[(4) - (6)].intAndDouble));
    // $$ = new constraint_goal(E_HOLDDURING,$5,NULL,$4->double_value(),$3->double_value());delete $3;delete $4;
;}
    break;

  case 201:
#line 2099 "pddl+.yacc"
    {
    WARNING("We don't deal with constraints as part of the goal (c_constraint_goal).\n");
    FAKE_DELETE((yyvsp[(3) - (5)].intAndDouble));
    // $$ = new constraint_goal(E_HOLDAFTER,$4,NULL,0.0,$3->double_value());delete $3;
;}
    break;

  case 202:
#line 2114 "pddl+.yacc"
    {

    (yyval.signedPredicates) = new SignedPredicates();
    (yyval.signedPredicates)->push_back((yyvsp[(1) - (1)].signedPredicate));
    //FAKE_DELETE($1);
;}
    break;

  case 203:
#line 2121 "pddl+.yacc"
    {
    /*Because we eventually have to support negative preconditions for
     * an action, we are going to have to support this rule. That
     * means that we are going to have that the
     * \rule{c_goal_descriptor} has a more elaborate type than simply
     * \type{Proposition}.*/

    (yyval.signedPredicates) = (yyvsp[(3) - (4)].signedPredicates);
    
    for(SignedPredicates::iterator p = (yyval.signedPredicates)->begin()
	    ; p != (yyval.signedPredicates)->end()
	    ; p++){
	(*p)->changeSign();
	
	if((*p)->getName() == "="){
	    problem.domain.requires.equality = true;
	} else {
	    problem.domain.requires.negativePreconditions = true;
	}
    }
    
    WARNING("We do not support negative preconditions, or equality"<<endl
	    <<" reasoning. So get the \"NOT\" out of the goal.");
;}
    break;

  case 204:
#line 2146 "pddl+.yacc"
    {
    WARNING("We only support one conjunction in the goal.\n");
    (yyval.signedPredicates) = 0;
    //$$= new conj_goal($3);
;}
    break;

  case 205:
#line 2152 "pddl+.yacc"
    {
    (yyval.signedPredicates) = 0;
    WARNING("We do not support disjunction in goals (c_goal_descriptor).\n");
//     $$= new disj_goal($3);
//     requires(E_DISJUNCTIVE_PRECONDS);
    problem.domain.requires.disjunctivePreconditions = true;
;}
    break;

  case 206:
#line 2160 "pddl+.yacc"
    {
    (yyval.signedPredicates) = 0;
    WARNING("We do not support implications in goals (c_goal_descriptor).\n");

    
    problem.domain.requires.disjunctivePreconditions = true;
//     $$= new imply_goal($3,$4);
//     requires(E_DISJUNCTIVE_PRECONDS);
;}
    break;

  case 207:
#line 2171 "pddl+.yacc"
    {
    (yyval.signedPredicates) = 0;
    WARNING("We do not support quantification in goals (c_goal_descriptor).\n");
    
    //$$= new qfied_goal($2,$4,$6,current_analysis->var_tab_stack.pop());
;}
    break;

  case 208:
#line 2178 "pddl+.yacc"
    {
    //$$= new comparison($2,$3,$4);

    WARNING("We do not support fluents (c_goal_descriptor).\n");
    (yyval.signedPredicates) = 0;
    problem.domain.requires.fluents = true;
    
    //requires(E_FLUENTS);
;}
    break;

  case 209:
#line 2200 "pddl+.yacc"
    {
    /*I have decided to parse the goal directly.*/
//     problem.goal.push_back();
//     $$=$1;
//     $1->push_back($2);
    assert((yyvsp[(1) - (2)].signedPredicates));
    
    (yyval.signedPredicates) = (yyvsp[(1) - (2)].signedPredicates);
    (yyval.signedPredicates)->insert((yyval.signedPredicates)->begin(), (yyvsp[(2) - (2)].signedPredicates)->begin(), (yyvsp[(2) - (2)].signedPredicates)->end());
    FAKE_DELETE((yyvsp[(2) - (2)].signedPredicates));
;}
    break;

  case 210:
#line 2212 "pddl+.yacc"
    {
    /*New list of facts.*/
    (yyval.signedPredicates) = new SignedPredicates();
;}
    break;

  case 211:
#line 2222 "pddl+.yacc"
    {;}
    break;

  case 212:
#line 2224 "pddl+.yacc"
    {;}
    break;

  case 213:
#line 2230 "pddl+.yacc"
    {
    //$$=$1;
;}
    break;

  case 214:
#line 2233 "pddl+.yacc"
    {
    //$$=$1;
    ;}
    break;

  case 215:
#line 2242 "pddl+.yacc"
    {// $$=E_FORALL; 
//     current_analysis->var_tab_stack.push(
// 	current_analysis->buildForallTab());
;}
    break;

  case 216:
#line 2252 "pddl+.yacc"
    {// $$=E_EXISTS;
//     current_analysis->var_tab_stack.push(
// 	current_analysis->buildExistsTab());
;}
    break;

  case 217:
#line 2262 "pddl+.yacc"
    {
    /*It may indeed no longer be the case that
     * \at{c_parameter_symbols} (\union{parameters}) we have all
     * constants.*/

    (yyval.signedPredicate) = NULL;

    /*Are all the parameters to the predicate symbol \type{Constant}?*/
    bool allConstant = true;
    for(Parameters::iterator p = (yyvsp[(3) - (4)].parameters)->begin(); allConstant && p != (yyvsp[(3) - (4)].parameters)->end(); p++){
	if(!dynamic_cast<Constant*>(*p)){
	    allConstant = false;
	}
    }

    /*If all the arguments are constant make a predicate. Otherwise
     * make a proposition. */
    if(allConstant){
	(yyval.signedPredicate) = new SignedProposition(*(yyvsp[(2) - (4)].predicateName), *(yyvsp[(3) - (4)].parameters));
    } else {
	(yyval.signedPredicate) = new SignedPredicate(*(yyvsp[(2) - (4)].predicateName), *(yyvsp[(3) - (4)].parameters));
    }

    (yyval.signedPredicate)->makePositive();
    
    FAKE_DELETE((yyvsp[(2) - (4)].predicateName));
    FAKE_DELETE((yyvsp[(3) - (4)].parameters));

    assert((yyval.signedPredicate));
;}
    break;

  case 218:
#line 2300 "pddl+.yacc"
    {
    WARNING("We do not support derived propositions or predicates (c_derived_proposition).\n");

    //$$ = NULL;
    // $$ = new proposition($2,$3);
;}
    break;

  case 219:
#line 2312 "pddl+.yacc"
    {
    Constants constants;
    for(Parameters::iterator p = (yyvsp[(3) - (4)].parameters)->begin(); p != (yyvsp[(3) - (4)].parameters)->end(); p++){
	assert(dynamic_cast<Constant*>(*p));

	constants.push_back(*dynamic_cast<Constant*>(*p));

	/*It is up to the signed proposition to delete the constant symbol.*/
	//FAKE_DELETE(*p);
    }
    
    
    (yyval.signedProposition) = new SignedProposition(*(yyvsp[(2) - (4)].predicateName),constants);
    
    FAKE_DELETE((yyvsp[(3) - (4)].parameters));
    FAKE_DELETE((yyvsp[(2) - (4)].predicateName));

    //$$=new proposition($2,$3);
;}
    break;

  case 220:
#line 2338 "pddl+.yacc"
    {;}
    break;

  case 221:
#line 2340 "pddl+.yacc"
    {
    yyerrok;
    //$$=NULL;
    UNRECOVERABLE_ERROR("Empty predicate declaration (c_predicates).");
    //log_error(E_FATAL,"Syntax error in (:predicates ...)");
;}
    break;

  case 222:
#line 2352 "pddl+.yacc"
    {
    WARNING("Don't support functions (c_functions_def).\n");
    //$$= $3;
;}
    break;

  case 223:
#line 2357 "pddl+.yacc"
    {
    WARNING("Don't support functions (c_functions_def).\n");
    yyerrok;
    //$$=NULL;
    WARNING("Syntax error in (:functions ...).");
//     log_error(E_FATAL,"Syntax error in (:functions ...)");
;}
    break;

  case 224:
#line 2370 "pddl+.yacc"
    {
    WARNING("Don't support constraints (c_constraints_def).\n");
    //$$ = $3;
;}
    break;

  case 225:
#line 2375 "pddl+.yacc"
    {
    yyerrok;
    
    WARNING("Don't support constraints (c_constraints_def).\n");
    //$$=NULL;
    WARNING("Syntax error in (:constraints ...).");
    
    //log_error(E_FATAL,"Syntax error in (:constraints ...)");
;}
    break;

  case 226:
#line 2390 "pddl+.yacc"
    {
    WARNING("Don't support constraints (c_constraints_def).\n");
    //$$ = $3;
;}
    break;

  case 227:
#line 2395 "pddl+.yacc"
    {
    WARNING("Don't support constraints (c_constraints_def).\n");
    yyerrok;
    
    WARNING("Syntax error in (:constraints ...).");
    //$$=NULL;
    //log_error(E_FATAL,"Syntax error in (:constraints ...)");
;}
    break;

  case 228:
#line 2411 "pddl+.yacc"
    {
    //$$=$1; $$->push_back($2);
;}
    break;

  case 229:
#line 2414 "pddl+.yacc"
    {
    //$$= new structure_store; $$->push_back($1);
    ;}
    break;

  case 230:
#line 2422 "pddl+.yacc"
    {
    VERBOSER(1, "Parsed action definition (c_structure_def).\n");
;}
    break;

  case 231:
#line 2425 "pddl+.yacc"
    {
    WARNING("We do not support domain events (c_structure_def).");
    // $$= $1; requires(E_TIME);
    ;}
    break;

  case 232:
#line 2429 "pddl+.yacc"
    {
    WARNING("We do not support domain processes (c_structure_def).");
    // $$= $1; requires(E_TIME);
    ;}
    break;

  case 233:
#line 2433 "pddl+.yacc"
    {
    WARNING("We do not support durative actions (c_structure_def).");
    // $$= $1; requires(E_DURATIVE_ACTIONS);
    ;}
    break;

  case 234:
#line 2437 "pddl+.yacc"
    {
    WARNING("We do not support durative rules (c_structure_def).");
    // $$= $1; requires(E_DERIVED_PREDICATES);
    ;}
    break;

  case 235:
#line 2446 "pddl+.yacc"
    {

    WARNING("We do not support derived predicates.\n");
//     $$= 0; 
//     current_analysis->var_tab_stack.push(
// 	current_analysis->buildRuleTab());
;}
    break;

  case 236:
#line 2463 "pddl+.yacc"
    {
    WARNING("We do not support derived predicates.\n");
    //$$ = new derivation_rule($3,$4,current_analysis->var_tab_stack.pop());
;}
    break;

  case 237:
#line 2480 "pddl+.yacc"
    {
    /*Parses a string that looks somewhat like this :: */
    /*( :action STRING
        :parameters ( c_typed_var_list )
	:precondition (c_pre_goal_descriptor)
	:effect (c_effect)) */

    /*In the case that the action cost is determined by grounding a predicate symbol.*/
    if(costEvaluator != 0){
	VERBOSER(26, "Parsed action with variable total cost ::"<<*costEvaluator<<endl);
	
	problem.domain.addAction(*(yyvsp[(3) - (12)].str), *(yyvsp[(6) - (12)].arguments), *(yyvsp[(9) - (12)].signedPredicates), *(yyvsp[(11) - (12)].signedPredicates), costEvaluator);
    }
    /*The default cost of an action is 1.*/
    else if(totalCost == 0 ){
	VERBOSER(26, "Parsed action with total cost ::"<<totalCost<<endl);
	
	problem.domain.addAction(*(yyvsp[(3) - (12)].str), *(yyvsp[(6) - (12)].arguments), *(yyvsp[(9) - (12)].signedPredicates), *(yyvsp[(11) - (12)].signedPredicates));
    } else {
	VERBOSER(26, "Parsed action with total cost ::"<<totalCost<<endl);
	
	problem.domain.addAction(*(yyvsp[(3) - (12)].str), *(yyvsp[(6) - (12)].arguments), *(yyvsp[(9) - (12)].signedPredicates), *(yyvsp[(11) - (12)].signedPredicates), totalCost);
    }
    

    totalCost = 0;
    costEvaluator = 0;
    
    FAKE_DELETE((yyvsp[(3) - (12)].str));
    FAKE_DELETE((yyvsp[(6) - (12)].arguments));
    FAKE_DELETE((yyvsp[(9) - (12)].signedPredicates));
    FAKE_DELETE((yyvsp[(11) - (12)].signedPredicates));
    
//     $$= current_analysis->buildAction(current_analysis->op_tab.symbol_put($3),
// 				    $6,$9,$11,
// 				    current_analysis->var_tab_stack.pop()); delete [] $3;
;}
    break;

  case 238:
#line 2518 "pddl+.yacc"
    {
    yyerrok;
    //$$= NULL;
    WARNING("Buggered action description (c_action_def).\n");
    // log_error(E_FATAL,"Syntax error in action declaration.");
;}
    break;

  case 239:
#line 2537 "pddl+.yacc"
    {
    WARNING("We do not support event definitions (c_event_def).\n");
//     $$= current_analysis->buildEvent(current_analysis->op_tab.symbol_put($3),
// 				  $6,$9,$11,
// 				  current_analysis->var_tab_stack.pop()); delete [] $3;
;}
    break;

  case 240:
#line 2545 "pddl+.yacc"
    {
    yyerrok; 
    //log_error(E_FATAL,"Syntax error in event declaration.");
    //$$= NULL;
    WARNING("Syntax error in event declaration.\n");
;}
    break;

  case 241:
#line 2563 "pddl+.yacc"
    {
/*     $$= current_analysis->buildProcess(current_analysis->op_tab.symbol_put($3), */
/* 				    $6,$9,$11, */
/* 				    current_analysis->var_tab_stack.pop()); delete [] $3; */
;}
    break;

  case 242:
#line 2569 "pddl+.yacc"
    {
    yyerrok;
    //$$= NULL;
    WARNING("No support for processes.\n");
    
    //log_error(E_FATAL,"Syntax error in process declaration.");
;}
    break;

  case 243:
#line 2588 "pddl+.yacc"
    {
/*     $$= $10; */
/*     $$->name= current_analysis->op_tab.symbol_put($3); */
/*     $$->symtab= current_analysis->var_tab_stack.pop(); */
/*     $$->parameters= $6; */
/*     $$->dur_constraint= $9;  */
/*     delete [] $3; */
;}
    break;

  case 244:
#line 2598 "pddl+.yacc"
    {
    yyerrok;
    WARNING("Empty durative action.\n");
    //log_error(E_FATAL,"Syntax error in durative-action declaration.");
    //$$= NULL;
;}
    break;

  case 245:
#line 2610 "pddl+.yacc"
    {// $$=$1; $$->effects=$3;
;}
    break;

  case 246:
#line 2613 "pddl+.yacc"
    {// $$=$1; $$->precondition=$3;
;}
    break;

  case 247:
#line 2615 "pddl+.yacc"
    {
    // $$= current_analysis->buildDurativeAction();
    ;}
    break;

  case 248:
#line 2628 "pddl+.yacc"
    { // $$=$1;
    WARNING("We do not support durative actions (c_da_gd).\n");
;}
    break;

  case 249:
#line 2632 "pddl+.yacc"
    {
    WARNING("We do not support durative actions (c_da_gd).\n");
    // $$= new conj_goal($3);
;}
    break;

  case 250:
#line 2642 "pddl+.yacc"
    {
    WARNING("We do not support durative actions (c_da_gds).\n");
    // $$=$1; $$->push_back($2);
;}
    break;

  case 251:
#line 2647 "pddl+.yacc"
    {
    WARNING("We do not support durative actions. (c_da_gds)\n");
    // $$= new goal_list;
;}
    break;

  case 252:
#line 2657 "pddl+.yacc"
    {// $$= new timed_goal($3,E_AT_START);
;}
    break;

  case 253:
#line 2660 "pddl+.yacc"
    {// $$= new timed_goal($3,E_AT_END);
;}
    break;

  case 254:
#line 2663 "pddl+.yacc"
    {// $$= new timed_goal($3,E_OVER_ALL);
;}
    break;

  case 255:
#line 2666 "pddl+.yacc"
    {// timed_goal * tg = dynamic_cast<timed_goal *>($4);
//     $$ = new timed_goal(new preference($3,tg->clearGoal()),tg->getTime());
//     delete tg;
//     requires(E_PREFERENCES);
;}
    break;

  case 256:
#line 2672 "pddl+.yacc"
    {// $$ = new preference($3);requires(E_PREFERENCES);
;}
    break;

  case 257:
#line 2679 "pddl+.yacc"
    {
//     $$= 0; current_analysis->var_tab_stack.push(
// 	current_analysis->buildOpTab());
;}
    break;

  case 258:
#line 2688 "pddl+.yacc"
    {

    (yyval.typeOfSymbols) = (yyvsp[(3) - (4)].typeOfSymbols);
    //problem.domain.addConstants(*$3);
    //FAKE_DELETE($3);
    
    //$$=$3;
;}
    break;

  case 259:
#line 2702 "pddl+.yacc"
    {
    (yyval.typeOfTypes)=(yyvsp[(3) - (4)].typeOfTypes);
    problem.domain.requires.typing = true;
    //requires(E_TYPING);
;}
    break;

  case 260:
#line 2717 "pddl+.yacc"
    {
    
    problem.name = *(yyvsp[(5) - (12)].str);
    problem.domainName = *(yyvsp[(9) - (12)].str);

    FAKE_DELETE((yyvsp[(5) - (12)].str));
    FAKE_DELETE((yyvsp[(9) - (12)].str));
;}
    break;

  case 261:
#line 2725 "pddl+.yacc"
    {
    yyerrok;
    WARNING("syntax error in _problem_ definition (c_problem).\n");
;}
    break;

  case 262:
#line 2734 "pddl+.yacc"
    {

    VERBOSER(1, "Problem requirement parsed.\n");
;}
    break;

  case 263:
#line 2738 "pddl+.yacc"
    {
    problem.setObjects((yyvsp[(1) - (2)].typeOfSymbols));
;}
    break;

  case 264:
#line 2741 "pddl+.yacc"
    {

    VERBOSER(1, "Parsed starting state :: "<<*(yyvsp[(1) - (2)].signedPredicates)<<endl);
    
    problem.setStartingState(*(yyvsp[(1) - (2)].signedPredicates));
    
    for_each((yyvsp[(1) - (2)].signedPredicates)->begin(), (yyvsp[(1) - (2)].signedPredicates)->end(), delete_pointers<SignedPredicate>());
    FAKE_DELETE((yyvsp[(1) - (2)].signedPredicates));
;}
    break;

  case 265:
#line 2750 "pddl+.yacc"
    {
    
    problem.setGoal(*(yyvsp[(1) - (2)].signedPredicates));

    /*Clean up.*/
    for_each((yyvsp[(1) - (2)].signedPredicates)->begin(), (yyvsp[(1) - (2)].signedPredicates)->end(), delete_pointers<SignedPredicate>());
    FAKE_DELETE((yyvsp[(1) - (2)].signedPredicates));
    
;}
    break;

  case 266:
#line 2760 "pddl+.yacc"
    {
    WARNING("We do not accept constraints (c_constraints_probdef).\n");
;}
    break;

  case 267:
#line 2763 "pddl+.yacc"
    {
    
    WARNING("We do not accept constraints (c_constraints_probdef).\n");
    //$$=$2;
    //$$->metric= $1;
;}
    break;

  case 268:
#line 2769 "pddl+.yacc"
    {
    //$$=$2;
    //$$->length= $1;
;}
    break;

  case 269:
#line 2773 "pddl+.yacc"
    {

    VERBOSER(1, "Currently we are only able to handle one problem at a time.");
    //problem = Problem();
    
    //$$=new problem;
     ;}
    break;

  case 270:
#line 2786 "pddl+.yacc"
    {
    (yyval.typeOfSymbols) = (yyvsp[(3) - (4)].typeOfSymbols);
;}
    break;

  case 271:
#line 2792 "pddl+.yacc"
    {
    //$$=$3;

    (yyval.signedPredicates) = (yyvsp[(3) - (4)].signedPredicates);
;}
    break;

  case 272:
#line 2801 "pddl+.yacc"
    {
    VERBOSER(1, "We parsed \":goals\" keyword.\n");
    //$$ = current_analysis->buildOpTab();
    
;}
    break;

  case 273:
#line 2811 "pddl+.yacc"
    {
    /*We have at \at{c_goals} a placeholder for \":goals\" keyword.*/
    
    (yyval.signedPredicates)=(yyvsp[(3) - (4)].signedPredicates);
    
    //delete $2;
;}
    break;

  case 274:
#line 2825 "pddl+.yacc"
    {
    WARNING("We do not support metric specifications.\n");
    //$$= new metric_spec($3,$4);
;}
    break;

  case 275:
#line 2830 "pddl+.yacc"
    {
    yyerrok;
    WARNING("We do not support metric specifications.\n");
    WARNING("Syntax error in metric declaration.");
    
    //log_error(E_FATAL,"Syntax error in metric declaration.");
    //$$= NULL;
;}
    break;

  case 276:
#line 2844 "pddl+.yacc"
    {
    /* Just calls \rule{c_length_field} which is specific to
     * \global{problem}'s \member{serialPlanLength} and
     * \member{parallelPlanLength}*/
    
    //$$= $3;
;}
    break;

  case 277:
#line 2856 "pddl+.yacc"
    {
    problem.serialPlanLength = (yyvsp[(2) - (2)].integer);
    //$$= new length_spec(E_SERIAL,$2);
;}
    break;

  case 278:
#line 2860 "pddl+.yacc"
    {
    problem.parallelPlanLength = (yyvsp[(2) - (5)].integer);
    //$$= new length_spec(E_PARALLEL,$2);
;}
    break;

  case 279:
#line 2869 "pddl+.yacc"
    {
    
    WARNING("We do not support optimisation conditions (c_optimization).\n");
    //$$= E_MINIMIZE;
;}
    break;

  case 280:
#line 2874 "pddl+.yacc"
    {
    
    WARNING("We do not support optimisation conditions (c_optimization).\n");
    //$$= E_MAXIMIZE;
   ;}
    break;

  case 281:
#line 2885 "pddl+.yacc"
    {
    // $$= $2;
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
;}
    break;

  case 282:
#line 2889 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
    // $$= $1;
    ;}
    break;

  case 283:
#line 2893 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
    FAKE_DELETE((yyvsp[(1) - (1)].intAndDouble));
    // $$= $1;
    ;}
    break;

  case 284:
#line 2898 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
    // $$= new special_val_expr(E_TOTAL_TIME);
    ;}
    break;

  case 285:
#line 2903 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
    // $$ = new violation_term($3);
;}
    break;

  case 286:
#line 2907 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_ground_f_exp).\n");
    // $$= new special_val_expr(E_TOTAL_TIME);
;}
    break;

  case 287:
#line 2916 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_exp).\n");
    // $$= new plus_expression($2,$3);
;}
    break;

  case 288:
#line 2920 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_exp).\n");
    // $$= new minus_expression($2,$3);
;}
    break;

  case 289:
#line 2924 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_exp).\n");
    // $$= new mul_expression($2,$3);
;}
    break;

  case 290:
#line 2928 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_exp).\n");
    // $$= new div_expression($2,$3);
;}
    break;

  case 291:
#line 2938 "pddl+.yacc"
    {
    // $$ = $1;
    WARNING("We do not support function expressions (c_binary_ground_f_pexps).\n");
;}
    break;

  case 292:
#line 2943 "pddl+.yacc"
    {
    // $$ = new plus_expression($1,$2);
    WARNING("We do not support function expressions (c_binary_ground_f_pexps).\n");
;}
    break;

  case 293:
#line 2952 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_mexps).\n");
    //$$ = $1;
;}
    break;

  case 294:
#line 2957 "pddl+.yacc"
    {
    WARNING("We do not support function expressions (c_binary_ground_f_mexps).\n");
    // $$ = new mul_expression($1,$2);
;}
    break;

  case 295:
#line 2982 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires EQUALITY (c_require_key).");
    problem.domain.requires.equality = true;
;}
    break;

  case 296:
#line 2987 "pddl+.yacc"
    {
    VERBOSER(18, "Domain requires ACTION_COSTS (c_require_key).")
    problem.domain.requires.actionCosts = true;
;}
    break;

  case 297:
#line 2991 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires STRIPS (c_require_key).");
    problem.domain.requires.strips = true;
  ;}
    break;

  case 298:
#line 2995 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires TYPES (c_require_key).");
    problem.domain.requires.typing = true;
  ;}
    break;

  case 299:
#line 2999 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires NEGATIVE-PRECONDITIONS (c_require_key).");
    problem.domain.requires.negativePreconditions = true;
;}
    break;

  case 300:
#line 3003 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires DISJUNCTIVE-PRECONDITIONS (c_require_key).");  
    problem.domain.requires.disjunctivePreconditions = true;
;}
    break;

  case 301:
#line 3007 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires EXISTENTIAL-PRECONDITIONS (c_require_key).");  
    problem.domain.requires.existentiallyQuantifiedPreconditions = true;
  ;}
    break;

  case 302:
#line 3011 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires UNIVERSAL-PRECONDITIONS (c_require_key).");  
    problem.domain.requires.universallyQuantifiedPreconditions = true;
  ;}
    break;

  case 303:
#line 3015 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires CONDITIONAL-EFFECTS (c_require_key).");  
    problem.domain.requires.conditionalEffects = true;
  ;}
    break;

  case 304:
#line 3019 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires FLUENTS (c_require_key).");  
    problem.domain.requires.fluents = true;
  ;}
    break;

  case 305:
#line 3023 "pddl+.yacc"
    { 
    VERBOSER(1, "Domain requires DURATIVE-ACTIONS (c_require_key).");  
    problem.domain.requires.durativeActions = true;
;}
    break;

  case 306:
#line 3027 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires TIM, FLUENTS, DURATIVE-ACTIONS (c_require_key).");  
    problem.domain.requires.time = true;
    problem.domain.requires.fluents = true;
    problem.domain.requires.durativeActions = true;
  ;}
    break;

  case 307:
#line 3033 "pddl+.yacc"
    { 
    VERBOSER(1, "Domain requires STRIPS, TYPING, NEGATIVE-PRECONDITIONS,"<<endl
	     <<"DISJUNCTIVE-PRECONDITIONS, EQUALITY, EXISTENTIAL-QUANTIFICATION"<<endl
	     <<"UNIVERSAL-QUANTIFICATION, CONDITIONAL-EFFECTS (c_require_key).");  
    problem.domain.requires.strips = true;
    problem.domain.requires.typing = true;
    problem.domain.requires.negativePreconditions = true;
    problem.domain.requires.disjunctivePreconditions = true;
    problem.domain.requires.equality = true;
    problem.domain.requires.existentiallyQuantifiedPreconditions = true;
    problem.domain.requires.universallyQuantifiedPreconditions = true;
    problem.domain.requires.conditionalEffects = true;
;}
    break;

  case 308:
#line 3046 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires existential quantification (c_require_key).");
    problem.domain.requires.existentiallyQuantifiedPreconditions = true;
    problem.domain.requires.universallyQuantifiedPreconditions = true;
  ;}
    break;

  case 309:
#line 3051 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires duration inequalities (c_require_key).");
    problem.domain.requires.durationInequalities = true;
;}
    break;

  case 310:
#line 3055 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires continuous effects (c_require_key).");
    problem.domain.requires.continuousEffects = true;
  ;}
    break;

  case 311:
#line 3059 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires derived predicates (c_require_key).");
    problem.domain.requires.derivedPredicates = true;
  ;}
    break;

  case 312:
#line 3063 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires timed literals (c_require_key).");
    problem.domain.requires.timedInitialLiterals = true;
  ;}
    break;

  case 313:
#line 3067 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires preferences (c_require_key).");
    problem.domain.requires.preferences = true;
  ;}
    break;

  case 314:
#line 3071 "pddl+.yacc"
    {
    VERBOSER(1, "Domain requires constants (c_require_key).");
    problem.domain.requires.constraints = true;
  ;}
    break;

  case 315:
#line 3075 "pddl+.yacc"
    {
    WARNING("Unrecognised requirements key in domain specification (c_require_key).");
;}
    break;


/* Line 1267 of yacc.c.  */
#line 5242 "pddl+.cc"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
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
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse look-ahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse look-ahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
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


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

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
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEOF && yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}


#line 3081 "pddl+.yacc"


    /*Postfix for files generated by bison.*/


#include <cstdio>
#include <iostream>
int line_no= 1;
using std::istream;
#include "lex.yy.cc"

namespace VAL
{
    yyFlexLexer* yfl;
}


void initialiseLexer(istream* arg_yyin, ostream* arg_yyout)
{
    VAL::yfl = new yyFlexLexer(arg_yyin, arg_yyout);
}

void deleteLexer()
{
    FAKE_DELETE(VAL::yfl);
}

int yyerror(char * s)
{
    return 0;
}

int yylex()
{
    return VAL::yfl->yylex();
}

