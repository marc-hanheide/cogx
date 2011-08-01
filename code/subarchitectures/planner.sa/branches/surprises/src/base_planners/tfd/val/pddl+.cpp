
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C
   
      Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

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
#define YYBISON_VERSION "2.4.1"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Copy the first part of user declarations.  */

/* Line 189 of yacc.c  */
#line 17 "pddl+.yacc"

/*
Error reporting:
Intention is to provide error token on most bracket expressions,
so synchronisation can occur on next CLOSE_BRAC.
Hence error should be generated for innermost expression containing error.
Expressions which cause errors return a NULL values, and parser
always attempts to carry on.
This won't behave so well if CLOSE_BRAC is missing.

Naming conventions:
Generally, the names should be similar to the PDDL2.1 spec.
During development, they have also been based on older PDDL specs,
older PDDL+ and TIM parsers, and this shows in places.

All the names of fields in the semantic value type begin with t_
Corresponding categories in the grammar begin with c_
Corresponding classes have no prefix.

PDDL grammar       yacc grammar      type of corresponding semantic val.  

thing+             c_things          thing_list
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

#include "ptree.h"
#include "parse_error.h"

#define YYDEBUG 1 

int yyerror(char *);


extern int yylex();

using namespace VAL;



/* Line 189 of yacc.c  */
#line 125 "pddl+.cpp"

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
     ACTION = 284,
     PROCESS = 285,
     EVENT = 286,
     DURATIVE_ACTION = 287,
     DERIVED = 288,
     CONSTANTS = 289,
     PREDS = 290,
     FUNCTIONS = 291,
     TYPES = 292,
     ARGS = 293,
     PRE = 294,
     CONDITION = 295,
     PREFERENCE = 296,
     START_PRE = 297,
     END_PRE = 298,
     EFFECTS = 299,
     INITIAL_EFFECT = 300,
     FINAL_EFFECT = 301,
     INVARIANT = 302,
     DURATION = 303,
     AT_START = 304,
     AT_END = 305,
     OVER_ALL = 306,
     AND = 307,
     OR = 308,
     EXISTS = 309,
     FORALL = 310,
     IMPLY = 311,
     NOT = 312,
     WHEN = 313,
     EITHER = 314,
     PROBLEM = 315,
     FORDOMAIN = 316,
     INITIALLY = 317,
     OBJECTS = 318,
     GOALS = 319,
     EQ = 320,
     LENGTH = 321,
     SERIAL = 322,
     PARALLEL = 323,
     METRIC = 324,
     MINIMIZE = 325,
     MAXIMIZE = 326,
     HASHT = 327,
     DURATION_VAR = 328,
     TOTAL_TIME = 329,
     INCREASE = 330,
     DECREASE = 331,
     SCALE_UP = 332,
     SCALE_DOWN = 333,
     ASSIGN = 334,
     GREATER = 335,
     GREATEQ = 336,
     LESS = 337,
     LESSEQ = 338,
     Q = 339,
     COLON = 340,
     ALWAYS = 341,
     SOMETIME = 342,
     WITHIN = 343,
     ATMOSTONCE = 344,
     SOMETIMEAFTER = 345,
     SOMETIMEBEFORE = 346,
     ALWAYSWITHIN = 347,
     HOLDDURING = 348,
     HOLDAFTER = 349,
     ISVIOLATED = 350,
     BOGUS = 351,
     NAME = 352,
     FUNCTION_SYMBOL = 353,
     INTVAL = 354,
     FLOATVAL = 355,
     AT_TIME = 356,
     PLUS = 357,
     HYPHEN = 358,
     DIV = 359,
     MUL = 360,
     UMINUS = 361
   };
#endif



#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
{

/* Line 214 of yacc.c  */
#line 68 "pddl+.yacc"

    parse_category* t_parse_category;

    effect_lists* t_effect_lists;
    effect* t_effect;
    simple_effect* t_simple_effect;
    cond_effect*   t_cond_effect;
    forall_effect* t_forall_effect;
    timed_effect* t_timed_effect;

    quantifier t_quantifier;
    metric_spec*  t_metric;
    optimization t_optimization;

    symbol* t_symbol;
    var_symbol*   t_var_symbol;
    pddl_type*    t_type;
    pred_symbol*  t_pred_symbol;
    func_symbol*  t_func_symbol;
    const_symbol* t_const_symbol;

    parameter_symbol_list* t_parameter_symbol_list;
    var_symbol_list* t_var_symbol_list;
    const_symbol_list* t_const_symbol_list;
    pddl_type_list* t_type_list;

    proposition* t_proposition;
    pred_decl* t_pred_decl;
    pred_decl_list* t_pred_decl_list;
    func_decl* t_func_decl;
    func_decl_list* t_func_decl_list;

    goal* t_goal;
    con_goal * t_con_goal;
    goal_list* t_goal_list;

    func_term* t_func_term;
    assignment* t_assignment;
    expression* t_expression;
    num_expression* t_num_expression;
    assign_op t_assign_op;
    comparison_op t_comparison_op;

    structure_def* t_structure_def;
    structure_store* t_structure_store;

    action* t_action_def;
    event* t_event_def;
    process* t_process_def;
    durative_action* t_durative_action_def;
    derivation_rule* t_derivation_rule;

    problem* t_problem;
    length_spec* t_length_spec;

    domain* t_domain;    

    pddl_req_flag t_pddl_req_flag;

    plan* t_plan;
    plan_step* t_step;

    int ival;
    double fval;

    char* cp;
    int t_dummy;

    var_symbol_table * vtab;



/* Line 214 of yacc.c  */
#line 340 "pddl+.cpp"
} YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif


/* Copy the second part of user declarations.  */


/* Line 264 of yacc.c  */
#line 352 "pddl+.cpp"

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
YYID (int yyi)
#else
static int
YYID (yyi)
    int yyi;
#endif
{
  return yyi;
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
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
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
# define YYSTACK_RELOCATE(Stack_alloc, Stack)				\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack_alloc, Stack, yysize);			\
	Stack = &yyptr->Stack_alloc;					\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  17
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   869

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  107
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  121
/* YYNRULES -- Number of rules.  */
#define YYNRULES  324
/* YYNRULES -- Number of states.  */
#define YYNSTATES  740

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   361

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
     105,   106
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     5,     7,     9,    15,    20,    23,    26,
      29,    32,    35,    38,    40,    45,    50,    55,    58,    59,
      62,    64,    69,    73,    75,    77,    79,    81,    84,    85,
      90,    94,    96,   101,   106,   108,   112,   113,   118,   123,
     125,   128,   129,   132,   133,   138,   143,   145,   148,   152,
     153,   155,   157,   159,   161,   166,   168,   170,   173,   174,
     177,   178,   185,   188,   191,   194,   195,   200,   203,   206,
     209,   210,   212,   214,   216,   218,   220,   225,   227,   229,
     231,   233,   236,   239,   242,   243,   248,   253,   258,   266,
     272,   274,   276,   279,   280,   285,   290,   296,   302,   306,
     311,   313,   315,   317,   319,   322,   325,   328,   329,   335,
     341,   347,   353,   359,   365,   371,   376,   379,   380,   382,
     385,   387,   389,   395,   401,   407,   413,   418,   425,   435,
     445,   447,   449,   451,   453,   456,   457,   462,   464,   469,
     471,   479,   485,   491,   497,   503,   509,   515,   520,   526,
     532,   538,   544,   546,   548,   554,   560,   562,   564,   566,
     571,   576,   578,   583,   588,   590,   592,   594,   596,   598,
     600,   602,   607,   615,   620,   626,   631,   639,   641,   646,
     652,   657,   665,   668,   669,   674,   680,   682,   685,   686,
     691,   699,   704,   709,   714,   720,   725,   731,   737,   744,
     751,   757,   759,   764,   769,   774,   780,   788,   794,   797,
     798,   801,   802,   804,   806,   808,   810,   815,   820,   825,
     830,   835,   840,   845,   850,   855,   860,   865,   868,   870,
     872,   874,   876,   878,   880,   882,   888,   901,   906,   919,
     924,   937,   942,   954,   959,   963,   967,   968,   970,   975,
     978,   979,   984,   989,   994,  1000,  1005,  1007,  1009,  1011,
    1013,  1015,  1017,  1019,  1021,  1023,  1025,  1027,  1029,  1031,
    1033,  1035,  1037,  1039,  1041,  1043,  1045,  1047,  1052,  1057,
    1070,  1076,  1079,  1082,  1085,  1088,  1091,  1094,  1097,  1098,
    1103,  1108,  1110,  1115,  1121,  1126,  1134,  1140,  1146,  1148,
    1150,  1154,  1156,  1158,  1160,  1165,  1169,  1173,  1177,  1181,
    1185,  1187,  1190,  1192,  1195,  1198,  1202,  1206,  1207,  1211,
    1213,  1218,  1220,  1225,  1227
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int16 yyrhs[] =
{
     108,     0,    -1,   109,    -1,   210,    -1,   223,    -1,     3,
       7,   111,   110,     4,    -1,     3,     7,   111,     1,    -1,
     112,   110,    -1,   209,   110,    -1,   208,   110,    -1,   190,
     110,    -1,   191,   110,    -1,   192,   110,    -1,   194,    -1,
       3,     8,    97,     4,    -1,     3,     9,   113,     4,    -1,
       3,     9,     1,     4,    -1,   113,   207,    -1,    -1,   115,
     114,    -1,   115,    -1,     3,   116,   122,     4,    -1,     3,
       1,     4,    -1,    97,    -1,    65,    -1,    97,    -1,    97,
      -1,   119,   120,    -1,    -1,     3,   121,   122,     4,    -1,
       3,     1,     4,    -1,    97,    -1,   123,   103,   135,   122,
      -1,   123,   103,   133,   122,    -1,   123,    -1,    84,   129,
     123,    -1,    -1,   126,   103,   135,   124,    -1,   126,   103,
     133,   124,    -1,   126,    -1,   131,   125,    -1,    -1,   132,
     126,    -1,    -1,   136,   103,   135,   127,    -1,   136,   103,
     133,   127,    -1,   136,    -1,   128,   131,    -1,   128,    84,
     130,    -1,    -1,    97,    -1,    97,    -1,    97,    -1,    97,
      -1,     3,    59,   137,     4,    -1,    97,    -1,    97,    -1,
     136,   134,    -1,    -1,   137,   135,    -1,    -1,   138,     3,
      65,   171,   170,     4,    -1,   138,   164,    -1,   138,   163,
      -1,   138,   139,    -1,    -1,     3,   101,   138,     4,    -1,
     142,   140,    -1,   166,   140,    -1,   165,   140,    -1,    -1,
     145,    -1,   162,    -1,   161,    -1,   166,    -1,   165,    -1,
       3,    52,   144,     4,    -1,   143,    -1,   161,    -1,   162,
      -1,   167,    -1,   144,   161,    -1,   144,   162,    -1,   144,
     167,    -1,    -1,     3,    52,   140,     4,    -1,     3,    52,
       1,     4,    -1,     3,    52,   147,     4,    -1,     3,   185,
       3,   122,     4,   146,     4,    -1,     3,    58,   203,   146,
       4,    -1,   148,    -1,   167,    -1,   147,   146,    -1,    -1,
       3,    49,   149,     4,    -1,     3,    50,   149,     4,    -1,
       3,    75,   171,   169,     4,    -1,     3,    76,   171,   169,
       4,    -1,     3,     1,     4,    -1,     3,    52,   151,     4,
      -1,   150,    -1,   161,    -1,   162,    -1,   152,    -1,   151,
     161,    -1,   151,   162,    -1,   151,   152,    -1,    -1,     3,
      79,   171,   155,     4,    -1,     3,    75,   171,   155,     4,
      -1,     3,    76,   171,   155,     4,    -1,     3,    77,   171,
     155,     4,    -1,     3,    78,   171,   155,     4,    -1,     3,
      75,   171,   169,     4,    -1,     3,    76,   171,   169,     4,
      -1,     3,    52,   154,     4,    -1,   154,   153,    -1,    -1,
     156,    -1,    84,    73,    -1,   170,    -1,   171,    -1,     3,
     102,   155,   155,     4,    -1,     3,   103,   155,   155,     4,
      -1,     3,   105,   155,   155,     4,    -1,     3,   104,   155,
     155,     4,    -1,     3,    52,   160,     4,    -1,     3,   158,
      84,    73,   159,     4,    -1,     3,    49,     3,   158,    84,
      73,   159,     4,     4,    -1,     3,    50,     3,   158,    84,
      73,   159,     4,     4,    -1,    83,    -1,    81,    -1,    65,
      -1,   168,    -1,   160,   157,    -1,    -1,     3,    57,   187,
       4,    -1,   187,    -1,     3,    57,   189,     4,    -1,   189,
      -1,     3,   185,     3,   122,     4,   141,     4,    -1,     3,
      58,   181,   140,     4,    -1,     3,    79,   171,   168,     4,
      -1,     3,    75,   171,   168,     4,    -1,     3,    76,   171,
     168,     4,    -1,     3,    77,   171,   168,     4,    -1,     3,
      78,   171,   168,     4,    -1,     3,   103,   168,     4,    -1,
       3,   102,   168,   168,     4,    -1,     3,   103,   168,   168,
       4,    -1,     3,   105,   168,   168,     4,    -1,     3,   104,
     168,   168,     4,    -1,   170,    -1,   171,    -1,     3,   105,
      72,   168,     4,    -1,     3,   105,   168,    72,     4,    -1,
      72,    -1,    99,    -1,   100,    -1,     3,    98,   128,     4,
      -1,     3,    97,   128,     4,    -1,    98,    -1,     3,    98,
     128,     4,    -1,     3,    97,   128,     4,    -1,    98,    -1,
      80,    -1,    81,    -1,    82,    -1,    83,    -1,    65,    -1,
     178,    -1,     3,    52,   182,     4,    -1,     3,   185,     3,
     122,     4,   174,     4,    -1,     3,    41,   180,     4,    -1,
       3,    41,    97,   180,     4,    -1,     3,    52,   177,     4,
      -1,     3,   185,     3,   122,     4,   176,     4,    -1,   180,
      -1,     3,    41,   180,     4,    -1,     3,    41,    97,   180,
       4,    -1,     3,    52,   177,     4,    -1,     3,   185,     3,
     122,     4,   176,     4,    -1,   177,   175,    -1,    -1,     3,
      41,   181,     4,    -1,     3,    41,    97,   181,     4,    -1,
     181,    -1,   179,   180,    -1,    -1,     3,    52,   179,     4,
      -1,     3,   185,     3,   122,     4,   180,     4,    -1,     3,
      50,   181,     4,    -1,     3,    86,   181,     4,    -1,     3,
      87,   181,     4,    -1,     3,    88,   170,   181,     4,    -1,
       3,    89,   181,     4,    -1,     3,    90,   181,   181,     4,
      -1,     3,    91,   181,   181,     4,    -1,     3,    92,   170,
     181,   181,     4,    -1,     3,    93,   170,   170,   181,     4,
      -1,     3,    94,   170,   181,     4,    -1,   187,    -1,     3,
      57,   181,     4,    -1,     3,    52,   183,     4,    -1,     3,
      53,   183,     4,    -1,     3,    56,   181,   181,     4,    -1,
       3,   184,     3,   122,     4,   181,     4,    -1,     3,   173,
     168,   168,     4,    -1,   182,   174,    -1,    -1,   183,   181,
      -1,    -1,   185,    -1,   186,    -1,    55,    -1,    54,    -1,
       3,   117,   128,     4,    -1,     3,   117,   122,     4,    -1,
       3,   118,   128,     4,    -1,     3,    35,   114,     4,    -1,
       3,    35,     1,     4,    -1,     3,    36,   119,     4,    -1,
       3,    36,     1,     4,    -1,     3,    28,   180,     4,    -1,
       3,    28,     1,     4,    -1,     3,    28,   175,     4,    -1,
       3,    28,     1,     4,    -1,   194,   195,    -1,   195,    -1,
     198,    -1,   199,    -1,   200,    -1,   201,    -1,   197,    -1,
      33,    -1,     3,   196,   188,   181,     4,    -1,     3,    29,
      97,   206,     3,   122,     4,    39,   174,    44,   141,     4,
      -1,     3,    29,     1,     4,    -1,     3,    31,    97,   206,
       3,   122,     4,    39,   181,    44,   141,     4,    -1,     3,
      31,     1,     4,    -1,     3,    30,    97,   206,     3,   122,
       4,    39,   181,    44,   153,     4,    -1,     3,    30,     1,
       4,    -1,     3,    32,    97,   206,     3,   122,     4,    48,
     157,   202,     4,    -1,     3,    32,     1,     4,    -1,   202,
      44,   146,    -1,   202,    40,   203,    -1,    -1,   205,    -1,
       3,    52,   204,     4,    -1,   204,   203,    -1,    -1,     3,
      49,   181,     4,    -1,     3,    50,   181,     4,    -1,     3,
      51,   181,     4,    -1,     3,    41,    97,   205,     4,    -1,
       3,    41,   205,     4,    -1,    38,    -1,    10,    -1,    11,
      -1,    14,    -1,    13,    -1,    15,    -1,    16,    -1,    17,
      -1,    19,    -1,    20,    -1,    22,    -1,    21,    -1,    12,
      -1,    18,    -1,    23,    -1,    24,    -1,    25,    -1,    26,
      -1,    27,    -1,    28,    -1,    97,    -1,     3,    34,   124,
       4,    -1,     3,    37,   127,     4,    -1,     3,     7,     3,
      60,    97,     4,     3,    61,    97,     4,   211,     4,    -1,
       3,     7,     3,    60,     1,    -1,   112,   211,    -1,   212,
     211,    -1,   213,   211,    -1,   215,   211,    -1,   193,   211,
      -1,   216,   211,    -1,   217,   211,    -1,    -1,     3,    63,
     124,     4,    -1,     3,    62,   138,     4,    -1,    64,    -1,
       3,   214,   174,     4,    -1,     3,    69,   218,   219,     4,
      -1,     3,    69,     1,     4,    -1,     3,    66,    67,    99,
      68,    99,     4,    -1,     3,    66,    67,    99,     4,    -1,
       3,    66,    68,    99,     4,    -1,    70,    -1,    71,    -1,
       3,   220,     4,    -1,   172,    -1,   170,    -1,    74,    -1,
       3,    95,    97,     4,    -1,     3,    74,     4,    -1,   102,
     219,   221,    -1,   103,   219,   219,    -1,   105,   219,   222,
      -1,   104,   219,   219,    -1,   219,    -1,   219,   221,    -1,
     219,    -1,   219,   222,    -1,   224,   223,    -1,    21,   100,
     223,    -1,    21,    99,   223,    -1,    -1,   227,    85,   225,
      -1,   225,    -1,   226,     5,   227,     6,    -1,   226,    -1,
       3,    97,   125,     4,    -1,   100,    -1,    99,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   240,   240,   241,   242,   246,   248,   255,   256,   257,
     258,   260,   262,   264,   267,   272,   279,   286,   287,   292,
     294,   299,   301,   309,   317,   319,   327,   332,   334,   338,
     340,   347,   360,   368,   376,   388,   390,   396,   404,   413,
     418,   419,   423,   424,   432,   439,   448,   454,   456,   458,
     465,   471,   475,   479,   483,   488,   495,   500,   502,   506,
     508,   512,   517,   519,   521,   524,   528,   534,   535,   537,
     539,   548,   549,   550,   551,   552,   556,   557,   561,   563,
     565,   572,   573,   574,   576,   580,   582,   590,   592,   600,
     605,   608,   615,   616,   620,   622,   624,   628,   632,   639,
     640,   644,   646,   648,   655,   656,   657,   659,   664,   666,
     668,   670,   672,   677,   683,   689,   694,   695,   699,   700,
     702,   703,   707,   709,   711,   713,   718,   720,   723,   726,
     732,   733,   734,   742,   746,   749,   753,   758,   765,   770,
     775,   780,   785,   787,   789,   791,   793,   798,   800,   802,
     804,   806,   808,   809,   813,   815,   817,   823,   824,   827,
     830,   832,   850,   852,   854,   860,   861,   862,   863,   864,
     876,   878,   880,   887,   889,   891,   893,   897,   902,   904,
     906,   908,   915,   918,   922,   924,   926,   931,   934,   938,
     940,   943,   945,   947,   949,   951,   953,   955,   957,   959,
     961,   966,   968,   972,   974,   977,   980,   983,   989,   992,
     996,   999,  1003,  1004,  1008,  1015,  1022,  1027,  1032,  1037,
    1039,  1046,  1048,  1055,  1057,  1064,  1066,  1073,  1074,  1078,
    1079,  1080,  1081,  1082,  1086,  1092,  1101,  1112,  1119,  1130,
    1136,  1146,  1152,  1167,  1174,  1176,  1178,  1182,  1184,  1189,
    1192,  1196,  1198,  1200,  1202,  1207,  1212,  1217,  1218,  1220,
    1221,  1223,  1225,  1226,  1227,  1228,  1229,  1231,  1235,  1244,
    1247,  1250,  1252,  1254,  1256,  1258,  1260,  1266,  1270,  1275,
    1282,  1289,  1290,  1291,  1292,  1293,  1295,  1296,  1297,  1300,
    1303,  1306,  1309,  1313,  1315,  1322,  1325,  1329,  1336,  1337,
    1342,  1343,  1344,  1345,  1346,  1348,  1352,  1353,  1354,  1355,
    1359,  1360,  1365,  1366,  1372,  1375,  1377,  1380,  1384,  1388,
    1394,  1398,  1404,  1412,  1413
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
  "CONSTRAINTS", "ACTION", "PROCESS", "EVENT", "DURATIVE_ACTION",
  "DERIVED", "CONSTANTS", "PREDS", "FUNCTIONS", "TYPES", "ARGS", "PRE",
  "CONDITION", "PREFERENCE", "START_PRE", "END_PRE", "EFFECTS",
  "INITIAL_EFFECT", "FINAL_EFFECT", "INVARIANT", "DURATION", "AT_START",
  "AT_END", "OVER_ALL", "AND", "OR", "EXISTS", "FORALL", "IMPLY", "NOT",
  "WHEN", "EITHER", "PROBLEM", "FORDOMAIN", "INITIALLY", "OBJECTS",
  "GOALS", "EQ", "LENGTH", "SERIAL", "PARALLEL", "METRIC", "MINIMIZE",
  "MAXIMIZE", "HASHT", "DURATION_VAR", "TOTAL_TIME", "INCREASE",
  "DECREASE", "SCALE_UP", "SCALE_DOWN", "ASSIGN", "GREATER", "GREATEQ",
  "LESS", "LESSEQ", "Q", "COLON", "ALWAYS", "SOMETIME", "WITHIN",
  "ATMOSTONCE", "SOMETIMEAFTER", "SOMETIMEBEFORE", "ALWAYSWITHIN",
  "HOLDDURING", "HOLDAFTER", "ISVIOLATED", "BOGUS", "NAME",
  "FUNCTION_SYMBOL", "INTVAL", "FLOATVAL", "AT_TIME", "PLUS", "HYPHEN",
  "DIV", "MUL", "UMINUS", "$accept", "mystartsymbol", "c_domain",
  "c_preamble", "c_domain_name", "c_domain_require_def", "c_reqs",
  "c_pred_decls", "c_pred_decl", "c_new_pred_symbol", "c_pred_symbol",
  "c_init_pred_symbol", "c_func_decls", "c_func_decl", "c_new_func_symbol",
  "c_typed_var_list", "c_var_symbol_list", "c_typed_consts",
  "c_const_symbols", "c_new_const_symbols", "c_typed_types",
  "c_parameter_symbols", "c_declaration_var_symbol", "c_var_symbol",
  "c_const_symbol", "c_new_const_symbol", "c_either_type",
  "c_new_primitive_type", "c_primitive_type", "c_new_primitive_types",
  "c_primitive_types", "c_init_els", "c_timed_initial_literal",
  "c_effects", "c_effect", "c_a_effect", "c_p_effect", "c_p_effects",
  "c_conj_effect", "c_da_effect", "c_da_effects", "c_timed_effect",
  "c_a_effect_da", "c_p_effect_da", "c_p_effects_da", "c_f_assign_da",
  "c_proc_effect", "c_proc_effects", "c_f_exp_da", "c_binary_expr_da",
  "c_duration_constraint", "c_d_op", "c_d_value", "c_duration_constraints",
  "c_neg_simple_effect", "c_pos_simple_effect", "c_init_neg_simple_effect",
  "c_init_pos_simple_effect", "c_forall_effect", "c_cond_effect",
  "c_assignment", "c_f_exp", "c_f_exp_t", "c_number", "c_f_head",
  "c_ground_f_head", "c_comparison_op", "c_pre_goal_descriptor",
  "c_pref_con_goal", "c_pref_goal", "c_pref_con_goal_list",
  "c_pref_goal_descriptor", "c_constraint_goal_list", "c_constraint_goal",
  "c_goal_descriptor", "c_pre_goal_descriptor_list", "c_goal_list",
  "c_quantifier", "c_forall", "c_exists", "c_proposition",
  "c_derived_proposition", "c_init_proposition", "c_predicates",
  "c_functions_def", "c_constraints_def", "c_constraints_probdef",
  "c_structure_defs", "c_structure_def", "c_rule_head",
  "c_derivation_rule", "c_action_def", "c_event_def", "c_process_def",
  "c_durative_action_def", "c_da_def_body", "c_da_gd", "c_da_gds",
  "c_timed_gd", "c_args_head", "c_require_key", "c_domain_constants",
  "c_type_names", "c_problem", "c_problem_body", "c_objects",
  "c_initial_state", "c_goals", "c_goal_spec", "c_metric_spec",
  "c_length_spec", "c_optimization", "c_ground_f_exp",
  "c_binary_ground_f_exp", "c_binary_ground_f_pexps",
  "c_binary_ground_f_mexps", "c_plan", "c_step_t_d", "c_step_d", "c_step",
  "c_float", 0
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
     355,   356,   357,   358,   359,   360,   361
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,   107,   108,   108,   108,   109,   109,   110,   110,   110,
     110,   110,   110,   110,   111,   112,   112,   113,   113,   114,
     114,   115,   115,   116,   117,   117,   118,   119,   119,   120,
     120,   121,   122,   122,   122,   123,   123,   124,   124,   124,
     125,   125,   126,   126,   127,   127,   127,   128,   128,   128,
     129,   130,   131,   132,   133,   134,   135,   136,   136,   137,
     137,   138,   138,   138,   138,   138,   139,   140,   140,   140,
     140,   141,   141,   141,   141,   141,   142,   142,   143,   143,
     143,   144,   144,   144,   144,   145,   145,   146,   146,   146,
     146,   146,   147,   147,   148,   148,   148,   148,   148,   149,
     149,   150,   150,   150,   151,   151,   151,   151,   152,   152,
     152,   152,   152,   153,   153,   153,   154,   154,   155,   155,
     155,   155,   156,   156,   156,   156,   157,   157,   157,   157,
     158,   158,   158,   159,   160,   160,   161,   162,   163,   164,
     165,   166,   167,   167,   167,   167,   167,   168,   168,   168,
     168,   168,   168,   168,   169,   169,   169,   170,   170,   171,
     171,   171,   172,   172,   172,   173,   173,   173,   173,   173,
     174,   174,   174,   175,   175,   175,   175,   175,   176,   176,
     176,   176,   177,   177,   178,   178,   178,   179,   179,   180,
     180,   180,   180,   180,   180,   180,   180,   180,   180,   180,
     180,   181,   181,   181,   181,   181,   181,   181,   182,   182,
     183,   183,   184,   184,   185,   186,   187,   188,   189,   190,
     190,   191,   191,   192,   192,   193,   193,   194,   194,   195,
     195,   195,   195,   195,   196,   197,   198,   198,   199,   199,
     200,   200,   201,   201,   202,   202,   202,   203,   203,   204,
     204,   205,   205,   205,   205,   205,   206,   207,   207,   207,
     207,   207,   207,   207,   207,   207,   207,   207,   207,   207,
     207,   207,   207,   207,   207,   207,   207,   208,   209,   210,
     210,   211,   211,   211,   211,   211,   211,   211,   211,   212,
     213,   214,   215,   216,   216,   217,   217,   217,   218,   218,
     219,   219,   219,   219,   219,   219,   220,   220,   220,   220,
     221,   221,   222,   222,   223,   223,   223,   223,   224,   224,
     225,   225,   226,   227,   227
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     1,     1,     5,     4,     2,     2,     2,
       2,     2,     2,     1,     4,     4,     4,     2,     0,     2,
       1,     4,     3,     1,     1,     1,     1,     2,     0,     4,
       3,     1,     4,     4,     1,     3,     0,     4,     4,     1,
       2,     0,     2,     0,     4,     4,     1,     2,     3,     0,
       1,     1,     1,     1,     4,     1,     1,     2,     0,     2,
       0,     6,     2,     2,     2,     0,     4,     2,     2,     2,
       0,     1,     1,     1,     1,     1,     4,     1,     1,     1,
       1,     2,     2,     2,     0,     4,     4,     4,     7,     5,
       1,     1,     2,     0,     4,     4,     5,     5,     3,     4,
       1,     1,     1,     1,     2,     2,     2,     0,     5,     5,
       5,     5,     5,     5,     5,     4,     2,     0,     1,     2,
       1,     1,     5,     5,     5,     5,     4,     6,     9,     9,
       1,     1,     1,     1,     2,     0,     4,     1,     4,     1,
       7,     5,     5,     5,     5,     5,     5,     4,     5,     5,
       5,     5,     1,     1,     5,     5,     1,     1,     1,     4,
       4,     1,     4,     4,     1,     1,     1,     1,     1,     1,
       1,     4,     7,     4,     5,     4,     7,     1,     4,     5,
       4,     7,     2,     0,     4,     5,     1,     2,     0,     4,
       7,     4,     4,     4,     5,     4,     5,     5,     6,     6,
       5,     1,     4,     4,     4,     5,     7,     5,     2,     0,
       2,     0,     1,     1,     1,     1,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     2,     1,     1,
       1,     1,     1,     1,     1,     5,    12,     4,    12,     4,
      12,     4,    11,     4,     3,     3,     0,     1,     4,     2,
       0,     4,     4,     4,     5,     4,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     4,     4,    12,
       5,     2,     2,     2,     2,     2,     2,     2,     0,     4,
       4,     1,     4,     5,     4,     7,     5,     5,     1,     1,
       3,     1,     1,     1,     4,     3,     3,     3,     3,     3,
       1,     2,     1,     2,     2,     3,     3,     0,     3,     1,
       4,     1,     4,     1,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint16 yydefact[] =
{
     317,     0,     0,   324,   323,     0,     2,     3,     4,   317,
     319,   321,     0,     0,    41,   317,   317,     1,     0,   314,
       0,     0,     0,     0,    52,     0,    41,   316,   315,     0,
     318,     0,     0,     6,     0,     0,     0,     0,     0,     0,
      13,   228,   233,   229,   230,   231,   232,     0,     0,   322,
      40,   320,     0,   280,     0,     0,     0,     0,     0,     0,
       0,   234,    43,     0,     0,    58,     0,     5,     7,    10,
      11,    12,     0,   227,     9,     8,    14,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    53,     0,    39,    43,     0,     0,     0,    20,     0,
       0,     0,    46,     0,     0,     0,    16,    15,   257,   258,
     268,   260,   259,   261,   262,   263,   269,   264,   265,   267,
     266,   270,   271,   272,   273,   274,   275,   276,    17,   224,
       0,   188,   214,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,   223,   237,   256,     0,   241,     0,   239,
       0,   243,     0,   277,     0,    42,   220,     0,    23,    36,
     219,    19,   222,     0,   221,    27,   278,    55,     0,    57,
      24,    25,    36,     0,     0,   201,     0,     0,     0,     0,
       0,   157,   158,     0,     0,     0,     0,     0,     0,     0,
      36,    36,    36,    36,    36,     0,    56,    43,    43,    22,
       0,     0,    34,     0,    31,    36,    58,    58,     0,   211,
     211,   215,     0,     0,   169,   165,   166,   167,   168,    49,
       0,     0,   212,   213,   235,     0,   191,   189,   187,   192,
     193,     0,   195,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,    60,    38,    37,    50,    36,    21,     0,
      30,     0,    45,    44,   217,     0,     0,     0,     0,     0,
       0,   161,     0,   152,   153,    36,   288,   194,   196,   197,
       0,     0,   200,     0,     0,     0,     0,     0,     0,    35,
      36,    36,    29,   203,   210,   204,     0,   202,   216,     0,
      47,    49,    49,     0,     0,     0,     0,     0,     0,     0,
     288,   288,     0,   288,   288,   288,   288,   288,   198,   199,
       0,     0,     0,     0,     0,    54,    59,    33,    32,   205,
      51,    48,     0,     0,     0,     0,     0,     0,   207,     0,
       0,    65,    43,   291,     0,     0,     0,   281,   285,   279,
     282,   283,   284,   286,   287,   190,     0,     0,   170,   186,
       0,     0,     0,   246,   160,   159,     0,   147,     0,     0,
       0,     0,     0,     0,     0,   177,     0,     0,     0,     0,
       0,   298,   299,     0,     0,     0,   209,     0,     0,     0,
       0,     0,     0,   135,   132,   131,   130,     0,     0,   148,
     149,   151,   150,   206,   226,     0,   183,     0,   225,     0,
     290,    64,    63,    62,   139,   289,     0,     0,   294,     0,
     303,   164,   302,   301,     0,   292,     0,     0,     0,    36,
       0,     0,    71,    73,    72,    75,    74,   137,     0,     0,
       0,     0,     0,     0,     0,   242,     0,     0,     0,     0,
       0,    36,     0,     0,    26,    65,    49,   296,     0,   297,
       0,     0,    49,    49,     0,     0,     0,     0,     0,   293,
       0,   184,   171,   208,     0,     0,     0,     0,     0,   236,
     117,     0,     0,   240,   238,     0,     0,   126,   134,     0,
       0,   245,   247,     0,   244,    90,    91,     0,   173,   175,
     182,     0,     0,     0,     0,     0,     0,     0,     0,   305,
       0,     0,     0,     0,     0,     0,     0,   300,   185,     0,
       0,     0,     0,    70,    77,    78,    79,    70,    70,    80,
       0,     0,    70,    36,     0,     0,     0,     0,     0,     0,
     133,     0,     0,     0,     0,   250,     0,     0,     0,    93,
       0,     0,     0,     0,     0,     0,     0,   174,     0,   138,
       0,    66,   218,   295,   304,   163,   162,   310,   306,   307,
     309,   312,   308,     0,    86,    84,     0,     0,    85,    67,
      69,    68,   136,     0,     0,   115,   116,     0,   156,     0,
       0,     0,     0,   127,     0,     0,     0,     0,     0,     0,
       0,    98,     0,     0,   100,   103,   101,   102,     0,     0,
       0,     0,     0,     0,     0,     0,    36,     0,     0,    61,
     311,   313,   172,     0,     0,     0,   141,     0,     0,   113,
     114,     0,     0,     0,   255,   251,   252,   253,   248,   249,
     107,     0,     0,     0,     0,     0,    94,    95,    87,    92,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,   183,     0,   176,     0,    76,    81,    82,    83,     0,
       0,     0,     0,     0,   254,     0,     0,     0,     0,     0,
       0,    89,     0,   143,    96,   144,    97,   145,   146,   142,
       0,     0,     0,     0,    36,   140,     0,     0,   128,   129,
       0,    99,   106,   104,   105,     0,     0,     0,   118,   120,
     121,     0,     0,     0,     0,     0,     0,     0,   178,   180,
       0,   154,   155,     0,     0,     0,     0,   119,   109,   110,
     111,   112,   108,    88,   179,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,   181,   122,   123,   125,   124
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     5,     6,    35,    23,   300,    79,    97,    98,   159,
     219,   446,   100,   165,   205,   201,   202,    92,    25,    93,
     101,   259,   247,   321,   290,    94,   197,   169,   198,   102,
     278,   366,   401,   512,   421,   513,   514,   613,   422,   484,
     599,   485,   593,   594,   665,   595,   429,   524,   697,   698,
     353,   387,   529,   433,   515,   516,   402,   403,   517,   518,
     519,   530,   579,   263,   264,   413,   220,   347,   490,   608,
     440,   348,   178,   310,   349,   418,   255,   221,   468,   223,
     175,   104,   404,    37,    38,    39,   301,    40,    41,    66,
      42,    43,    44,    45,    46,   388,   481,   590,   482,   146,
     128,    47,    48,     7,   302,   303,   304,   336,   305,   306,
     307,   373,   557,   458,   558,   562,     8,     9,    10,    11,
      12
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -471
static const yytype_int16 yypact[] =
{
      53,    44,   -10,  -471,  -471,    34,  -471,  -471,  -471,   104,
    -471,   100,   125,    72,   139,   104,   104,  -471,   164,  -471,
     128,   214,   197,    70,  -471,   218,   139,  -471,  -471,   259,
    -471,   172,     2,  -471,   674,   269,   297,   297,   297,   297,
     301,  -471,  -471,  -471,  -471,  -471,  -471,   297,   297,  -471,
    -471,  -471,   310,  -471,   312,   539,   210,     4,    37,    57,
      67,  -471,   222,   238,   424,  -471,   351,  -471,  -471,  -471,
    -471,  -471,   423,  -471,  -471,  -471,  -471,   359,   366,   589,
     372,   568,   379,   396,   373,   398,   373,   414,   373,   425,
     373,  -471,   431,   334,   222,   438,    68,   453,   446,   462,
     276,   475,   -43,    -6,   472,   440,  -471,  -471,  -471,  -471,
    -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,
    -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,  -471,
     472,  -471,  -471,   472,   472,   231,   472,   472,   472,   231,
     231,   231,   481,  -471,  -471,  -471,   483,  -471,   510,  -471,
     516,  -471,   523,  -471,     9,  -471,  -471,   537,  -471,   464,
    -471,  -471,  -471,    77,  -471,  -471,  -471,  -471,     9,  -471,
    -471,  -471,   464,   635,   577,  -471,   489,   583,   357,   584,
     585,  -471,  -471,   472,   586,   472,   472,   472,   231,   472,
     464,   464,   464,   464,   464,   513,  -471,   222,   222,  -471,
     494,   617,   519,   620,  -471,   464,  -471,  -471,   622,  -471,
    -471,  -471,   472,   472,   105,  -471,  -471,  -471,  -471,  -471,
     116,   628,  -471,  -471,  -471,   630,  -471,  -471,  -471,  -471,
    -471,   631,  -471,   633,   640,   472,   472,   641,   646,   647,
     648,   649,   659,  -471,  -471,  -471,  -471,   464,  -471,     9,
    -471,   669,  -471,  -471,  -471,   363,   387,   472,   680,   122,
     471,  -471,   116,  -471,  -471,   464,   629,  -471,  -471,  -471,
     681,   690,  -471,   692,   553,   600,   654,   650,   132,  -471,
     464,   464,  -471,  -471,  -471,  -471,   693,  -471,  -471,   599,
    -471,  -471,  -471,   116,   116,   116,   116,   695,   697,   335,
     629,   629,   709,   629,   629,   629,   629,   629,  -471,  -471,
     710,   716,   472,   472,   717,  -471,  -471,  -471,  -471,  -471,
    -471,  -471,   188,   191,   116,    33,   116,   116,  -471,   472,
     245,  -471,   222,  -471,   266,   120,   716,  -471,  -471,  -471,
    -471,  -471,  -471,  -471,  -471,  -471,   435,   677,  -471,  -471,
     678,   679,   367,  -471,  -471,  -471,   720,  -471,   725,   726,
     727,   731,   740,   578,   746,  -471,   404,   747,   653,   655,
     749,  -471,  -471,    40,   751,    47,  -471,   753,   754,   755,
     754,   756,   757,  -471,  -471,  -471,  -471,   682,   325,  -471,
    -471,  -471,  -471,  -471,  -471,    52,  -471,   758,  -471,   205,
    -471,  -471,  -471,  -471,  -471,  -471,   176,   759,  -471,   336,
    -471,  -471,  -471,  -471,   760,  -471,   472,   761,   443,   464,
     180,   763,  -471,  -471,  -471,  -471,  -471,  -471,   178,   764,
     765,    -4,    -4,   474,   698,  -471,   767,   769,   692,   770,
     491,   464,   772,    39,  -471,  -471,  -471,  -471,   663,  -471,
     773,   676,  -471,  -471,    40,    40,    40,    40,   774,  -471,
     775,  -471,  -471,  -471,   776,   461,   778,   472,   779,  -471,
    -471,    39,    39,  -471,  -471,   699,   700,  -471,  -471,   116,
      31,  -471,  -471,   232,  -471,  -471,  -471,   781,  -471,  -471,
    -471,   782,   691,   783,   401,   231,   505,   192,   785,  -471,
     786,   194,   208,    40,    40,    40,    40,  -471,  -471,   716,
     787,   280,   788,   790,  -471,  -471,  -471,   790,   790,  -471,
      -6,   791,   790,   464,   508,    30,    30,   703,   721,   792,
    -471,    85,   472,   472,   472,  -471,   793,   795,   795,  -471,
     767,    39,    39,    39,    39,    39,   796,  -471,   797,  -471,
     798,  -471,  -471,  -471,  -471,  -471,  -471,    40,  -471,  -471,
    -471,    40,  -471,   799,  -471,  -471,    39,    39,  -471,  -471,
    -471,  -471,  -471,   800,   801,  -471,  -471,   696,  -471,   802,
     803,   116,   116,  -471,   330,   805,   806,   807,   808,   809,
     525,  -471,   445,   810,  -471,  -471,  -471,  -471,   811,   527,
     769,    63,    63,   116,   116,   116,   464,   588,   812,  -471,
    -471,  -471,  -471,   530,   116,   116,  -471,   754,    73,  -471,
    -471,   813,   814,   815,  -471,  -471,  -471,  -471,  -471,  -471,
    -471,    39,    39,    39,    39,    39,  -471,  -471,  -471,  -471,
     816,   544,   817,   818,   819,   820,   821,   822,   823,   824,
      89,  -471,   826,  -471,   271,  -471,  -471,  -471,  -471,   827,
     116,   737,   828,   829,  -471,   541,    59,    59,    59,    59,
      59,  -471,    73,  -471,  -471,  -471,  -471,  -471,  -471,  -471,
     769,   692,   830,   543,   464,  -471,   831,   832,  -471,  -471,
     428,  -471,  -471,  -471,  -471,   636,   766,   833,  -471,  -471,
    -471,   834,   836,   837,   838,   109,   839,   840,  -471,  -471,
     841,  -471,  -471,    59,    59,    59,    59,  -471,  -471,  -471,
    -471,  -471,  -471,  -471,  -471,   797,    59,    59,    59,    59,
     842,   843,   844,   845,   846,  -471,  -471,  -471,  -471,  -471
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -471,  -471,  -471,   304,  -471,   689,  -471,   732,  -471,  -471,
     748,  -471,  -471,  -471,  -471,  -170,   605,  -184,   835,   762,
     364,  -252,  -471,  -471,    38,  -471,   -18,  -471,  -136,  -471,
    -471,   408,  -471,  -200,  -374,  -471,  -471,  -471,  -471,  -379,
    -471,  -471,   316,  -471,  -471,   190,   333,  -471,  -255,  -471,
     426,   146,    -2,  -471,  -353,  -350,  -471,  -471,  -361,  -349,
    -430,  -209,  -308,  -131,  -246,  -471,  -471,  -310,   528,   135,
     211,  -471,  -471,   -56,   -89,  -471,   656,  -471,   -80,  -471,
    -362,  -471,   421,  -471,  -471,  -471,  -471,  -471,   825,  -471,
    -471,  -471,  -471,  -471,  -471,  -471,  -470,  -471,  -338,   -23,
    -471,  -471,  -471,  -471,   442,  -471,  -471,  -471,  -471,  -471,
    -471,  -471,  -327,  -471,   307,   306,   243,  -471,   847,  -471,
     849
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -71
static const yytype_int16 yytable[] =
{
      82,   142,   208,    53,   183,    83,   430,   486,   187,   188,
     189,   262,   195,   244,   245,   174,   427,   425,   427,   425,
     238,   239,   240,   241,   242,   423,   374,   423,   424,   426,
     424,   426,   207,   577,    17,   251,   260,   357,    85,   322,
     323,   177,   494,   409,   179,   180,   414,   184,   185,   186,
     173,    13,    26,   297,   167,    81,     1,   236,    87,   170,
     168,   384,   695,   148,    26,   150,   641,   152,    89,   157,
     600,    33,   531,    34,     2,    22,   260,   385,   203,   386,
     532,   533,   534,   535,   324,   325,   326,   327,   584,    15,
      16,   171,    81,   222,   231,   298,   233,   234,   235,    54,
     237,    84,   578,   427,   521,    20,   196,    18,   463,   -24,
     317,   318,   260,   281,   410,   356,   358,   359,   360,   260,
     629,   370,   228,   257,   258,     2,   288,   503,   504,   505,
     506,   261,   181,   182,    86,   578,   315,   261,   411,   181,
     182,    14,   316,   696,   416,   660,   270,   271,   367,   438,
     206,   427,     3,     4,    88,   427,   427,   261,   181,   182,
     427,   261,   181,   182,    90,   158,   284,   284,   286,   486,
     486,   261,   181,   182,   204,   427,   427,   559,   560,   561,
     447,   687,   585,   658,   596,   596,   681,   597,   597,   -24,
     371,   372,   354,   586,   497,   355,   552,   495,   555,   563,
     501,   502,   -24,     3,     4,    31,   289,   261,   181,   182,
      21,    80,   556,    81,   261,   181,   182,    18,   580,    24,
     639,   640,    49,   350,   351,   525,   526,     3,     4,   196,
     470,   280,   465,   536,   561,   132,    24,   466,   467,    95,
     361,    96,   412,   659,   448,   170,   362,   623,   363,   464,
     486,   427,    19,   471,   472,   427,   425,    32,    27,    28,
     656,    14,   442,   657,   423,    51,   377,   424,   426,    52,
     443,   491,   289,    67,   365,   289,   289,   171,   289,   163,
     164,   537,   538,   397,   539,    24,   417,   132,    24,    24,
     540,    24,   289,   643,   645,   601,   602,   603,   604,   605,
      34,   706,   444,   427,    72,    24,   445,   541,   542,   543,
     544,   545,   693,   569,    76,   694,    77,   570,   571,    91,
     614,   615,   573,   412,   412,   412,   412,   460,   466,   435,
     181,   182,   565,   368,   369,   132,   170,   466,   467,   439,
      68,    69,    70,    71,    55,   170,   566,   567,   543,   544,
     545,    74,    75,   574,   103,   566,   567,   543,   544,   545,
      81,   227,   105,   330,   550,   436,   173,   283,   171,   437,
     106,   531,   412,   412,   412,   412,   129,   171,   522,   532,
     533,   534,   487,   143,   365,   666,   667,   668,   669,   670,
     173,   285,   642,   644,   646,   647,   648,   331,   332,   333,
     144,   334,   147,   546,   335,   642,   644,   399,   400,   661,
     450,   145,   701,   702,   703,   704,   381,   382,   149,   383,
     700,   700,   700,   700,   700,    99,   412,   -28,   -28,   151,
     412,   451,   384,   452,   453,   153,   649,   154,   454,   455,
     456,   457,   156,   587,   588,   589,   346,   462,   385,    96,
     386,   686,    57,    58,    59,    60,    61,   160,   726,   727,
     728,   729,   510,   705,   511,   -70,   162,   700,   700,   700,
     700,   731,   732,   733,   734,   173,   375,   352,   477,   166,
     700,   700,   700,   700,   190,   466,   191,   376,   210,   211,
     132,   212,   213,   170,   363,   489,   360,   630,   291,   292,
     214,   176,   466,   631,   632,   633,   634,   635,   399,   551,
     170,   428,   575,   192,   710,   215,   216,   217,   218,   193,
     631,   632,   633,   634,   635,   171,   194,   652,   480,   628,
     483,   638,   171,   654,   655,   699,   699,   699,   699,   699,
      78,   199,   171,   -18,   690,   691,   363,   709,   200,   -18,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   291,   292,
     252,   253,   243,   293,   294,   295,   296,   475,   476,   621,
     622,   224,   699,   699,   699,   699,   225,   226,   229,   230,
     232,   246,   311,   107,   682,   699,   699,   699,   699,   108,
     109,   110,   111,   112,   113,   114,   115,   116,   117,   118,
     119,   120,   121,   122,   123,   124,   125,   126,   130,   395,
     131,   248,   249,   132,   250,   707,   254,   365,   130,   650,
     396,   265,   299,   132,   266,   267,   -18,   268,   130,   312,
     651,   291,   292,   132,   269,   272,   293,   294,   295,   672,
     273,   274,   275,   276,   133,   134,   135,   136,   137,   138,
     139,   140,   141,   277,   133,   134,   135,   136,   137,   138,
     139,   140,   141,   282,   133,   134,   135,   136,   137,   138,
     139,   140,   141,    55,   287,   308,   127,   209,   210,   211,
     132,   212,   213,   313,   309,    81,   320,   319,   314,   328,
     214,   329,    56,    57,    58,    59,    60,    61,    62,    63,
      64,    65,    36,   339,   345,   215,   216,   217,   218,   346,
     352,   378,   379,   380,   389,    36,    36,    36,    36,   390,
     391,   392,   171,   291,   292,   393,    36,    36,   713,   714,
     715,   716,   337,   338,   394,   340,   341,   342,   343,   344,
     398,   405,   406,   408,   407,   415,   419,   420,   428,   431,
     432,   441,   498,   449,   459,   461,   434,   469,   473,   474,
     480,   479,   483,   500,   488,   492,   581,   499,   507,   508,
     509,   520,   523,   527,   528,   547,   548,   549,   444,   553,
     554,   564,   568,   511,   582,   572,   583,   591,   592,   606,
     607,   618,   609,   612,   616,   617,   619,   620,   584,   687,
     624,   625,   626,   627,   636,   637,   653,   662,   663,   664,
     671,   673,   674,   675,   676,   677,   678,   679,   680,   684,
     161,   685,   688,   689,   708,   711,   712,   718,   719,   717,
     720,   721,   722,   723,   724,   725,   735,   736,   737,   738,
     739,   172,   279,   496,   598,   692,   155,   576,   364,   478,
     730,    50,   683,   493,   610,    73,   256,   611,    30,    29
};

static const yytype_uint16 yycheck[] =
{
      56,    81,   172,     1,   135,     1,   380,   437,   139,   140,
     141,   220,     3,   197,   198,   104,   378,   378,   380,   380,
     190,   191,   192,   193,   194,   378,   336,   380,   378,   378,
     380,   380,   168,     3,     0,   205,     3,     4,     1,   291,
     292,   130,     3,     3,   133,   134,   373,   136,   137,   138,
       3,     7,    14,   262,    97,     3,     3,   188,     1,    65,
     103,    65,     3,    86,    26,    88,     3,    90,     1,     1,
     540,     1,    41,     3,    21,     3,     3,    81,     1,    83,
      49,    50,    51,    52,   293,   294,   295,   296,     3,    99,
     100,    97,     3,   173,   183,   265,   185,   186,   187,    97,
     189,    97,    72,   465,   466,     5,    97,     3,   418,     4,
     280,   281,     3,   249,    74,   324,   325,   326,   327,     3,
     590,     1,   178,   212,   213,    21,     4,   454,   455,   456,
     457,    98,    99,   100,    97,    72,     4,    98,    98,    99,
     100,    97,   278,    84,    97,    72,   235,   236,   332,    97,
     168,   513,    99,   100,    97,   517,   518,    98,    99,   100,
     522,    98,    99,   100,    97,    97,   255,   256,   257,   599,
     600,    98,    99,   100,    97,   537,   538,   504,   505,   506,
       4,    72,    97,   613,   537,   538,    97,   537,   538,    84,
      70,    71,     4,   531,   446,     4,     4,   443,     4,   509,
     452,   453,    97,    99,   100,     8,    84,    98,    99,   100,
      85,     1,     4,     3,    98,    99,   100,     3,   526,    97,
     599,   600,     4,   312,   313,   471,   472,    99,   100,    97,
      52,   249,    52,     1,   561,    55,    97,    57,    58,     1,
     329,     3,   373,   617,    68,    65,     1,   585,     3,   419,
     680,   613,     9,    75,    76,   617,   617,    60,    15,    16,
     613,    97,    57,   613,   617,     6,   346,   617,   617,    97,
      65,   441,    84,     4,   330,    84,    84,    97,    84,     3,
       4,    49,    50,   363,    52,    97,   375,    55,    97,    97,
      58,    97,    84,   601,   602,   541,   542,   543,   544,   545,
       3,   680,    97,   665,     3,    97,   101,    75,    76,    77,
      78,    79,   665,   513,     4,   665,     4,   517,   518,    97,
     566,   567,   522,   454,   455,   456,   457,   416,    57,     4,
      99,   100,    52,    67,    68,    55,    65,    57,    58,   395,
      36,    37,    38,    39,     9,    65,    75,    76,    77,    78,
      79,    47,    48,   523,     3,    75,    76,    77,    78,    79,
       3,     4,     3,    28,   495,    40,     3,     4,    97,    44,
       4,    41,   503,   504,   505,   506,     4,    97,   467,    49,
      50,    51,   438,     4,   440,   631,   632,   633,   634,   635,
       3,     4,   601,   602,   603,   604,   605,    62,    63,    64,
       4,    66,     4,   483,    69,   614,   615,     3,     4,   618,
      74,    38,   667,   668,   669,   670,    49,    50,     4,    52,
     666,   667,   668,   669,   670,     1,   557,     3,     4,     4,
     561,    95,    65,    97,    98,     4,   606,   103,   102,   103,
     104,   105,     4,   532,   533,   534,     3,     4,    81,     3,
      83,   660,    29,    30,    31,    32,    33,     4,   713,   714,
     715,   716,     1,   672,     3,     4,     4,   713,   714,   715,
     716,   726,   727,   728,   729,     3,    41,     3,     4,     4,
     726,   727,   728,   729,     3,    57,     3,    52,    53,    54,
      55,    56,    57,    65,     3,     4,   705,    52,    97,    98,
      65,    61,    57,    75,    76,    77,    78,    79,     3,     4,
      65,     3,     4,     3,   684,    80,    81,    82,    83,     3,
      75,    76,    77,    78,    79,    97,     3,   607,     3,     4,
       3,     4,    97,     3,     4,   666,   667,   668,   669,   670,
       1,     4,    97,     4,     3,     4,     3,     4,    84,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    97,    98,
     206,   207,    59,   102,   103,   104,   105,   431,   432,   581,
     582,     4,   713,   714,   715,   716,    97,     4,     4,     4,
       4,    97,    39,     4,   650,   726,   727,   728,   729,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    50,    41,
      52,     4,   103,    55,     4,   681,     4,   683,    50,    41,
      52,     3,     3,    55,     4,     4,    97,     4,    50,    39,
      52,    97,    98,    55,     4,     4,   102,   103,   104,   105,
       4,     4,     4,     4,    86,    87,    88,    89,    90,    91,
      92,    93,    94,     4,    86,    87,    88,    89,    90,    91,
      92,    93,    94,     4,    86,    87,    88,    89,    90,    91,
      92,    93,    94,     9,     4,     4,    97,    52,    53,    54,
      55,    56,    57,    39,     4,     3,    97,     4,    48,     4,
      65,     4,    28,    29,    30,    31,    32,    33,    34,    35,
      36,    37,    23,     4,     4,    80,    81,    82,    83,     3,
       3,    44,    44,    44,     4,    36,    37,    38,    39,     4,
       4,     4,    97,    97,    98,     4,    47,    48,   102,   103,
     104,   105,   300,   301,     4,   303,   304,   305,   306,   307,
       4,     4,    99,     4,    99,     4,     3,     3,     3,     3,
       3,     3,    99,     4,     4,     4,    84,     4,     4,     4,
       3,    73,     3,    97,     4,     3,    73,     4,     4,     4,
       4,     3,     3,    84,    84,     4,     4,     4,    97,     4,
       4,     4,     4,     3,    73,     4,     4,     4,     3,     3,
       3,   105,     4,     4,     4,     4,     4,     4,     3,    72,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     3,
      98,     4,     4,     4,     4,     4,     4,     4,     4,    73,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,   103,   247,   445,   538,   665,    94,   524,   330,   433,
     725,    26,   651,   442,   557,    40,   210,   561,    21,    20
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,    21,    99,   100,   108,   109,   210,   223,   224,
     225,   226,   227,     7,    97,    99,   100,     0,     3,   223,
       5,    85,     3,   111,    97,   125,   131,   223,   223,   227,
     225,     8,    60,     1,     3,   110,   112,   190,   191,   192,
     194,   195,   197,   198,   199,   200,   201,   208,   209,     4,
     125,     6,    97,     1,    97,     9,    28,    29,    30,    31,
      32,    33,    34,    35,    36,    37,   196,     4,   110,   110,
     110,   110,     3,   195,   110,   110,     4,     4,     1,   113,
       1,     3,   180,     1,    97,     1,    97,     1,    97,     1,
      97,    97,   124,   126,   132,     1,     3,   114,   115,     1,
     119,   127,   136,     3,   188,     3,     4,     4,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28,    97,   207,     4,
      50,    52,    55,    86,    87,    88,    89,    90,    91,    92,
      93,    94,   185,     4,     4,    38,   206,     4,   206,     4,
     206,     4,   206,     4,   103,   126,     4,     1,    97,   116,
       4,   114,     4,     3,     4,   120,     4,    97,   103,   134,
      65,    97,   117,     3,   181,   187,    61,   181,   179,   181,
     181,    99,   100,   170,   181,   181,   181,   170,   170,   170,
       3,     3,     3,     3,     3,     3,    97,   133,   135,     4,
      84,   122,   123,     1,    97,   121,   133,   135,   122,    52,
      53,    54,    56,    57,    65,    80,    81,    82,    83,   117,
     173,   184,   185,   186,     4,    97,     4,     4,   180,     4,
       4,   181,     4,   181,   181,   181,   170,   181,   122,   122,
     122,   122,   122,    59,   124,   124,    97,   129,     4,   103,
       4,   122,   127,   127,     4,   183,   183,   181,   181,   128,
       3,    98,   168,   170,   171,     3,     4,     4,     4,     4,
     181,   181,     4,     4,     4,     4,     4,     4,   137,   123,
     133,   135,     4,     4,   181,     4,   181,     4,     4,    84,
     131,    97,    98,   102,   103,   104,   105,   168,   122,     3,
     112,   193,   211,   212,   213,   215,   216,   217,     4,     4,
     180,    39,    39,    39,    48,     4,   135,   122,   122,     4,
      97,   130,   128,   128,   168,   168,   168,   168,     4,     4,
      28,    62,    63,    64,    66,    69,   214,   211,   211,     4,
     211,   211,   211,   211,   211,     4,     3,   174,   178,   181,
     181,   181,     3,   157,     4,     4,   168,     4,   168,   168,
     168,   181,     1,     3,   175,   180,   138,   124,    67,    68,
       1,    70,    71,   218,   174,    41,    52,   185,    44,    44,
      44,    49,    50,    52,    65,    81,    83,   158,   202,     4,
       4,     4,     4,     4,     4,    41,    52,   185,     4,     3,
       4,   139,   163,   164,   189,     4,    99,    99,     4,     3,
      74,    98,   170,   172,   219,     4,    97,   181,   182,     3,
       3,   141,   145,   161,   162,   165,   166,   187,     3,   153,
     141,     3,     3,   160,    84,     4,    40,    44,    97,   180,
     177,     3,    57,    65,    97,   101,   118,     4,    68,     4,
      74,    95,    97,    98,   102,   103,   104,   105,   220,     4,
     181,     4,     4,   174,   122,    52,    57,    58,   185,     4,
      52,    75,    76,     4,     4,   158,   158,     4,   157,    73,
       3,   203,   205,     3,   146,   148,   167,   180,     4,     4,
     175,   122,     3,   189,     3,   171,   138,   128,    99,     4,
      97,   128,   128,   219,   219,   219,   219,     4,     4,     4,
       1,     3,   140,   142,   143,   161,   162,   165,   166,   167,
       3,   187,   181,     3,   154,   171,   171,    84,    84,   159,
     168,    41,    49,    50,    51,    52,     1,    49,    50,    52,
      58,    75,    76,    77,    78,    79,   185,     4,     4,     4,
     170,     4,     4,     4,     4,     4,     4,   219,   221,   219,
     219,   219,   222,   174,     4,    52,    75,    76,     4,   140,
     140,   140,     4,   140,   122,     4,   153,     3,    72,   169,
     169,    73,    73,     4,     3,    97,   205,   181,   181,   181,
     204,     4,     3,   149,   150,   152,   161,   162,   149,   147,
     203,   171,   171,   171,   171,   171,     3,     3,   176,     4,
     221,   222,     4,   144,   171,   171,     4,     4,   105,     4,
       4,   159,   159,   205,     4,     4,     4,     4,     4,   203,
      52,    75,    76,    77,    78,    79,     4,     4,     4,   146,
     146,     3,   168,   169,   168,   169,   168,   168,   168,   122,
      41,    52,   185,     4,     3,     4,   161,   162,   167,   141,
      72,   168,     4,     4,     4,   151,   171,   171,   171,   171,
     171,     4,   105,     4,     4,     4,     4,     4,     4,     4,
       4,    97,   180,   177,     3,     4,   168,    72,     4,     4,
       3,     4,   152,   161,   162,     3,    84,   155,   156,   170,
     171,   155,   155,   155,   155,   168,   146,   180,     4,     4,
     122,     4,     4,   102,   103,   104,   105,    73,     4,     4,
       4,     4,     4,     4,     4,     4,   155,   155,   155,   155,
     176,   155,   155,   155,   155,     4,     4,     4,     4,     4
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
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
#else
static void
yy_stack_print (yybottom, yytop)
    yytype_int16 *yybottom;
    yytype_int16 *yytop;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
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
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      YYFPRINTF (stderr, "\n");
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


/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*-------------------------.
| yyparse or yypush_parse.  |
`-------------------------*/

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
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       `yyss': related to states.
       `yyvs': related to semantic values.

       Refer to the stacks thru separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yytoken = 0;
  yyss = yyssa;
  yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */

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
	YYSTACK_RELOCATE (yyss_alloc, yyss);
	YYSTACK_RELOCATE (yyvs_alloc, yyvs);
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

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

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

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
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

/* Line 1455 of yacc.c  */
#line 240 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_domain); current_analysis->the_domain= (yyvsp[(1) - (1)].t_domain);;}
    break;

  case 3:

/* Line 1455 of yacc.c  */
#line 241 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_problem); current_analysis->the_problem= (yyvsp[(1) - (1)].t_problem);;}
    break;

  case 4:

/* Line 1455 of yacc.c  */
#line 242 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_plan); ;}
    break;

  case 5:

/* Line 1455 of yacc.c  */
#line 247 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(4) - (5)].t_domain); (yyval.t_domain)->name= (yyvsp[(3) - (5)].cp);delete [] (yyvsp[(3) - (5)].cp);;}
    break;

  case 6:

/* Line 1455 of yacc.c  */
#line 249 "pddl+.yacc"
    {yyerrok; (yyval.t_domain)=static_cast<domain*>(NULL);
       	log_error(E_FATAL,"Syntax error in domain"); ;}
    break;

  case 7:

/* Line 1455 of yacc.c  */
#line 255 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 8:

/* Line 1455 of yacc.c  */
#line 256 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->types= (yyvsp[(1) - (2)].t_type_list);;}
    break;

  case 9:

/* Line 1455 of yacc.c  */
#line 257 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->constants= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 10:

/* Line 1455 of yacc.c  */
#line 258 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->predicates= (yyvsp[(1) - (2)].t_pred_decl_list); ;}
    break;

  case 11:

/* Line 1455 of yacc.c  */
#line 260 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->functions= (yyvsp[(1) - (2)].t_func_decl_list); ;}
    break;

  case 12:

/* Line 1455 of yacc.c  */
#line 262 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain);
   										(yyval.t_domain)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 13:

/* Line 1455 of yacc.c  */
#line 264 "pddl+.yacc"
    {(yyval.t_domain)= new domain((yyvsp[(1) - (1)].t_structure_store)); ;}
    break;

  case 14:

/* Line 1455 of yacc.c  */
#line 267 "pddl+.yacc"
    {(yyval.cp)=(yyvsp[(3) - (4)].cp);;}
    break;

  case 15:

/* Line 1455 of yacc.c  */
#line 273 "pddl+.yacc"
    {
	// Stash in analysis object --- we need to refer to it during parse
	//   but domain object is not created yet,
	current_analysis->req |= (yyvsp[(3) - (4)].t_pddl_req_flag);
	(yyval.t_pddl_req_flag)=(yyvsp[(3) - (4)].t_pddl_req_flag);
    ;}
    break;

  case 16:

/* Line 1455 of yacc.c  */
#line 280 "pddl+.yacc"
    {yyerrok; 
       log_error(E_FATAL,"Syntax error in requirements declaration.");
       (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 17:

/* Line 1455 of yacc.c  */
#line 286 "pddl+.yacc"
    { (yyval.t_pddl_req_flag)= (yyvsp[(1) - (2)].t_pddl_req_flag) | (yyvsp[(2) - (2)].t_pddl_req_flag); ;}
    break;

  case 18:

/* Line 1455 of yacc.c  */
#line 287 "pddl+.yacc"
    { (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 19:

/* Line 1455 of yacc.c  */
#line 293 "pddl+.yacc"
    {(yyval.t_pred_decl_list)=(yyvsp[(2) - (2)].t_pred_decl_list); (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (2)].t_pred_decl));;}
    break;

  case 20:

/* Line 1455 of yacc.c  */
#line 295 "pddl+.yacc"
    {  (yyval.t_pred_decl_list)=new pred_decl_list;
           (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (1)].t_pred_decl)); ;}
    break;

  case 21:

/* Line 1455 of yacc.c  */
#line 300 "pddl+.yacc"
    {(yyval.t_pred_decl)= new pred_decl((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 22:

/* Line 1455 of yacc.c  */
#line 302 "pddl+.yacc"
    {yyerrok; 
        // hope someone makes this error someday
        log_error(E_FATAL,"Syntax error in predicate declaration.");
	(yyval.t_pred_decl)= NULL; ;}
    break;

  case 23:

/* Line 1455 of yacc.c  */
#line 310 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           				current_analysis->buildPredTab());
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 24:

/* Line 1455 of yacc.c  */
#line 317 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_ref("="); 
	      requires(E_EQUALITY); ;}
    break;

  case 25:

/* Line 1455 of yacc.c  */
#line 319 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 26:

/* Line 1455 of yacc.c  */
#line 327 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 27:

/* Line 1455 of yacc.c  */
#line 333 "pddl+.yacc"
    {(yyval.t_func_decl_list)=(yyvsp[(1) - (2)].t_func_decl_list); (yyval.t_func_decl_list)->push_back((yyvsp[(2) - (2)].t_func_decl));;}
    break;

  case 28:

/* Line 1455 of yacc.c  */
#line 334 "pddl+.yacc"
    { (yyval.t_func_decl_list)=new func_decl_list; ;}
    break;

  case 29:

/* Line 1455 of yacc.c  */
#line 339 "pddl+.yacc"
    {(yyval.t_func_decl)= new func_decl((yyvsp[(2) - (4)].t_func_symbol),(yyvsp[(3) - (4)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 30:

/* Line 1455 of yacc.c  */
#line 341 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in functor declaration.");
	 (yyval.t_func_decl)= NULL; ;}
    break;

  case 31:

/* Line 1455 of yacc.c  */
#line 348 "pddl+.yacc"
    { (yyval.t_func_symbol)=current_analysis->func_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           		current_analysis->buildFuncTab()); 
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 32:

/* Line 1455 of yacc.c  */
#line 361 "pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   ;}
    break;

  case 33:

/* Line 1455 of yacc.c  */
#line 369 "pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));    /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   ;}
    break;

  case 34:

/* Line 1455 of yacc.c  */
#line 377 "pddl+.yacc"
    {
       (yyval.t_var_symbol_list)= (yyvsp[(1) - (1)].t_var_symbol_list);
   ;}
    break;

  case 35:

/* Line 1455 of yacc.c  */
#line 389 "pddl+.yacc"
    {(yyval.t_var_symbol_list)=(yyvsp[(3) - (3)].t_var_symbol_list); (yyvsp[(3) - (3)].t_var_symbol_list)->push_front((yyvsp[(2) - (3)].t_var_symbol)); ;}
    break;

  case 36:

/* Line 1455 of yacc.c  */
#line 390 "pddl+.yacc"
    {(yyval.t_var_symbol_list)= new var_symbol_list; ;}
    break;

  case 37:

/* Line 1455 of yacc.c  */
#line 397 "pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list)); /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_const_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
   ;}
    break;

  case 38:

/* Line 1455 of yacc.c  */
#line 405 "pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list));
      delete (yyvsp[(4) - (4)].t_const_symbol_list);
      requires(E_TYPING);
   ;}
    break;

  case 39:

/* Line 1455 of yacc.c  */
#line 413 "pddl+.yacc"
    {(yyval.t_const_symbol_list)= (yyvsp[(1) - (1)].t_const_symbol_list);;}
    break;

  case 40:

/* Line 1455 of yacc.c  */
#line 418 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 41:

/* Line 1455 of yacc.c  */
#line 419 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 42:

/* Line 1455 of yacc.c  */
#line 423 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 43:

/* Line 1455 of yacc.c  */
#line 424 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 44:

/* Line 1455 of yacc.c  */
#line 433 "pddl+.yacc"
    {  
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
       (yyval.t_type_list)->splice((yyval.t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list)); /* Join lists */ 
       delete (yyvsp[(4) - (4)].t_type_list);                   /* Delete (now empty) list */
   ;}
    break;

  case 45:

/* Line 1455 of yacc.c  */
#line 440 "pddl+.yacc"
    {  
   // This parse needs to be excluded, we think (DPL&MF: 6/9/01)
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
       (yyval.t_type_list)->splice((yyvsp[(1) - (4)].t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list));
       delete (yyvsp[(4) - (4)].t_type_list);
   ;}
    break;

  case 46:

/* Line 1455 of yacc.c  */
#line 449 "pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(1) - (1)].t_type_list); ;}
    break;

  case 47:

/* Line 1455 of yacc.c  */
#line 455 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (2)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(2) - (2)].t_const_symbol)); ;}
    break;

  case 48:

/* Line 1455 of yacc.c  */
#line 457 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (3)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(3) - (3)].t_var_symbol)); ;}
    break;

  case 49:

/* Line 1455 of yacc.c  */
#line 458 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)= new parameter_symbol_list;;}
    break;

  case 50:

/* Line 1455 of yacc.c  */
#line 465 "pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.top()->symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 51:

/* Line 1455 of yacc.c  */
#line 471 "pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 52:

/* Line 1455 of yacc.c  */
#line 475 "pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 53:

/* Line 1455 of yacc.c  */
#line 479 "pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 54:

/* Line 1455 of yacc.c  */
#line 484 "pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(3) - (4)].t_type_list); ;}
    break;

  case 55:

/* Line 1455 of yacc.c  */
#line 489 "pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 56:

/* Line 1455 of yacc.c  */
#line 496 "pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 57:

/* Line 1455 of yacc.c  */
#line 501 "pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 58:

/* Line 1455 of yacc.c  */
#line 502 "pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 59:

/* Line 1455 of yacc.c  */
#line 507 "pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 60:

/* Line 1455 of yacc.c  */
#line 508 "pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 61:

/* Line 1455 of yacc.c  */
#line 513 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (6)].t_effect_lists);
	  (yyval.t_effect_lists)->assign_effects.push_back(new assignment((yyvsp[(4) - (6)].t_func_term),E_ASSIGN,(yyvsp[(5) - (6)].t_num_expression)));  
          requires(E_FLUENTS); 
	;}
    break;

  case 62:

/* Line 1455 of yacc.c  */
#line 518 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 63:

/* Line 1455 of yacc.c  */
#line 520 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 64:

/* Line 1455 of yacc.c  */
#line 522 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(2) - (2)].t_timed_effect)); ;}
    break;

  case 65:

/* Line 1455 of yacc.c  */
#line 524 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;;}
    break;

  case 66:

/* Line 1455 of yacc.c  */
#line 529 "pddl+.yacc"
    { requires(E_TIMED_INITIAL_LITERALS); 
   		(yyval.t_timed_effect)=new timed_initial_literal((yyvsp[(3) - (4)].t_effect_lists),(yyvsp[(2) - (4)].fval));;}
    break;

  case 67:

/* Line 1455 of yacc.c  */
#line 534 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->append_effects((yyvsp[(1) - (2)].t_effect_lists)); delete (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 68:

/* Line 1455 of yacc.c  */
#line 535 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (2)].t_cond_effect)); 
                                      requires(E_COND_EFFS);;}
    break;

  case 69:

/* Line 1455 of yacc.c  */
#line 537 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (2)].t_forall_effect));
                                      requires(E_COND_EFFS);;}
    break;

  case 70:

/* Line 1455 of yacc.c  */
#line 539 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists(); ;}
    break;

  case 71:

/* Line 1455 of yacc.c  */
#line 548 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 72:

/* Line 1455 of yacc.c  */
#line 549 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 73:

/* Line 1455 of yacc.c  */
#line 550 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 74:

/* Line 1455 of yacc.c  */
#line 551 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (1)].t_cond_effect));;}
    break;

  case 75:

/* Line 1455 of yacc.c  */
#line 552 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (1)].t_forall_effect));;}
    break;

  case 76:

/* Line 1455 of yacc.c  */
#line 556 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 77:

/* Line 1455 of yacc.c  */
#line 557 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 78:

/* Line 1455 of yacc.c  */
#line 562 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 79:

/* Line 1455 of yacc.c  */
#line 564 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 80:

/* Line 1455 of yacc.c  */
#line 566 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_FLUENTS);;}
    break;

  case 81:

/* Line 1455 of yacc.c  */
#line 572 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 82:

/* Line 1455 of yacc.c  */
#line 573 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 83:

/* Line 1455 of yacc.c  */
#line 574 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_FLUENTS); ;}
    break;

  case 84:

/* Line 1455 of yacc.c  */
#line 576 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 85:

/* Line 1455 of yacc.c  */
#line 581 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 86:

/* Line 1455 of yacc.c  */
#line 583 "pddl+.yacc"
    {yyerrok; (yyval.t_effect_lists)=NULL;
	 log_error(E_FATAL,"Syntax error in (and ...)");
	;}
    break;

  case 87:

/* Line 1455 of yacc.c  */
#line 591 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 88:

/* Line 1455 of yacc.c  */
#line 596 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);;}
    break;

  case 89:

/* Line 1455 of yacc.c  */
#line 601 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 90:

/* Line 1455 of yacc.c  */
#line 606 "pddl+.yacc"
    { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(1) - (1)].t_timed_effect)); ;}
    break;

  case 91:

/* Line 1455 of yacc.c  */
#line 609 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
          requires(E_FLUENTS); ;}
    break;

  case 92:

/* Line 1455 of yacc.c  */
#line 615 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 93:

/* Line 1455 of yacc.c  */
#line 616 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 94:

/* Line 1455 of yacc.c  */
#line 621 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_START);;}
    break;

  case 95:

/* Line 1455 of yacc.c  */
#line 623 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_END);;}
    break;

  case 96:

/* Line 1455 of yacc.c  */
#line 625 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 97:

/* Line 1455 of yacc.c  */
#line 629 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 98:

/* Line 1455 of yacc.c  */
#line 633 "pddl+.yacc"
    {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in timed effect"); ;}
    break;

  case 99:

/* Line 1455 of yacc.c  */
#line 639 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 100:

/* Line 1455 of yacc.c  */
#line 640 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 101:

/* Line 1455 of yacc.c  */
#line 645 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 102:

/* Line 1455 of yacc.c  */
#line 647 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 103:

/* Line 1455 of yacc.c  */
#line 649 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_FLUENTS);;}
    break;

  case 104:

/* Line 1455 of yacc.c  */
#line 655 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 105:

/* Line 1455 of yacc.c  */
#line 656 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 106:

/* Line 1455 of yacc.c  */
#line 657 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_FLUENTS); ;}
    break;

  case 107:

/* Line 1455 of yacc.c  */
#line 659 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 108:

/* Line 1455 of yacc.c  */
#line 665 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 109:

/* Line 1455 of yacc.c  */
#line 667 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 110:

/* Line 1455 of yacc.c  */
#line 669 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 111:

/* Line 1455 of yacc.c  */
#line 671 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 112:

/* Line 1455 of yacc.c  */
#line 673 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 113:

/* Line 1455 of yacc.c  */
#line 678 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 114:

/* Line 1455 of yacc.c  */
#line 684 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 115:

/* Line 1455 of yacc.c  */
#line 690 "pddl+.yacc"
    {(yyval.t_effect_lists) = (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 116:

/* Line 1455 of yacc.c  */
#line 694 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 117:

/* Line 1455 of yacc.c  */
#line 695 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 118:

/* Line 1455 of yacc.c  */
#line 699 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 119:

/* Line 1455 of yacc.c  */
#line 700 "pddl+.yacc"
    {(yyval.t_expression)= new special_val_expr(E_DURATION_VAR);
                    requires( E_DURATION_INEQUALITIES );;}
    break;

  case 120:

/* Line 1455 of yacc.c  */
#line 702 "pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 121:

/* Line 1455 of yacc.c  */
#line 703 "pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); ;}
    break;

  case 122:

/* Line 1455 of yacc.c  */
#line 708 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 123:

/* Line 1455 of yacc.c  */
#line 710 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 124:

/* Line 1455 of yacc.c  */
#line 712 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 125:

/* Line 1455 of yacc.c  */
#line 714 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 126:

/* Line 1455 of yacc.c  */
#line 719 "pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 127:

/* Line 1455 of yacc.c  */
#line 721 "pddl+.yacc"
    { (yyval.t_goal)= new timed_goal(new comparison((yyvsp[(2) - (6)].t_comparison_op),
        			new special_val_expr(E_DURATION_VAR),(yyvsp[(5) - (6)].t_expression)),E_AT_START); ;}
    break;

  case 128:

/* Line 1455 of yacc.c  */
#line 724 "pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_START);;}
    break;

  case 129:

/* Line 1455 of yacc.c  */
#line 727 "pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_END);;}
    break;

  case 130:

/* Line 1455 of yacc.c  */
#line 732 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_LESSEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 131:

/* Line 1455 of yacc.c  */
#line 733 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_GREATEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 132:

/* Line 1455 of yacc.c  */
#line 734 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 133:

/* Line 1455 of yacc.c  */
#line 742 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression); ;}
    break;

  case 134:

/* Line 1455 of yacc.c  */
#line 747 "pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 135:

/* Line 1455 of yacc.c  */
#line 749 "pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 136:

/* Line 1455 of yacc.c  */
#line 754 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 137:

/* Line 1455 of yacc.c  */
#line 759 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 138:

/* Line 1455 of yacc.c  */
#line 766 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 139:

/* Line 1455 of yacc.c  */
#line 771 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 140:

/* Line 1455 of yacc.c  */
#line 776 "pddl+.yacc"
    { (yyval.t_forall_effect)= new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop());;}
    break;

  case 141:

/* Line 1455 of yacc.c  */
#line 781 "pddl+.yacc"
    { (yyval.t_cond_effect)= new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)); ;}
    break;

  case 142:

/* Line 1455 of yacc.c  */
#line 786 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 143:

/* Line 1455 of yacc.c  */
#line 788 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 144:

/* Line 1455 of yacc.c  */
#line 790 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 145:

/* Line 1455 of yacc.c  */
#line 792 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 146:

/* Line 1455 of yacc.c  */
#line 794 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 147:

/* Line 1455 of yacc.c  */
#line 799 "pddl+.yacc"
    { (yyval.t_expression)= new uminus_expression((yyvsp[(3) - (4)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 148:

/* Line 1455 of yacc.c  */
#line 801 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 149:

/* Line 1455 of yacc.c  */
#line 803 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 150:

/* Line 1455 of yacc.c  */
#line 805 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 151:

/* Line 1455 of yacc.c  */
#line 807 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_FLUENTS); ;}
    break;

  case 152:

/* Line 1455 of yacc.c  */
#line 808 "pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 153:

/* Line 1455 of yacc.c  */
#line 809 "pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); requires(E_FLUENTS); ;}
    break;

  case 154:

/* Line 1455 of yacc.c  */
#line 814 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression(new special_val_expr(E_HASHT),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 155:

/* Line 1455 of yacc.c  */
#line 816 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression), new special_val_expr(E_HASHT)); ;}
    break;

  case 156:

/* Line 1455 of yacc.c  */
#line 818 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_HASHT); ;}
    break;

  case 157:

/* Line 1455 of yacc.c  */
#line 823 "pddl+.yacc"
    { (yyval.t_num_expression)=new int_expression((yyvsp[(1) - (1)].ival));   ;}
    break;

  case 158:

/* Line 1455 of yacc.c  */
#line 824 "pddl+.yacc"
    { (yyval.t_num_expression)=new float_expression((yyvsp[(1) - (1)].fval)); ;}
    break;

  case 159:

/* Line 1455 of yacc.c  */
#line 828 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 160:

/* Line 1455 of yacc.c  */
#line 831 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 161:

/* Line 1455 of yacc.c  */
#line 833 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 162:

/* Line 1455 of yacc.c  */
#line 851 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 163:

/* Line 1455 of yacc.c  */
#line 853 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 164:

/* Line 1455 of yacc.c  */
#line 855 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 165:

/* Line 1455 of yacc.c  */
#line 860 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATER; ;}
    break;

  case 166:

/* Line 1455 of yacc.c  */
#line 861 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATEQ; ;}
    break;

  case 167:

/* Line 1455 of yacc.c  */
#line 862 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESS; ;}
    break;

  case 168:

/* Line 1455 of yacc.c  */
#line 863 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESSEQ; ;}
    break;

  case 169:

/* Line 1455 of yacc.c  */
#line 864 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 170:

/* Line 1455 of yacc.c  */
#line 877 "pddl+.yacc"
    {(yyval.t_goal)= (yyvsp[(1) - (1)].t_goal);;}
    break;

  case 171:

/* Line 1455 of yacc.c  */
#line 879 "pddl+.yacc"
    {(yyval.t_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 172:

/* Line 1455 of yacc.c  */
#line 882 "pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);;}
    break;

  case 173:

/* Line 1455 of yacc.c  */
#line 888 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 174:

/* Line 1455 of yacc.c  */
#line 890 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 175:

/* Line 1455 of yacc.c  */
#line 892 "pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 176:

/* Line 1455 of yacc.c  */
#line 895 "pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 177:

/* Line 1455 of yacc.c  */
#line 898 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(1) - (1)].t_con_goal);;}
    break;

  case 178:

/* Line 1455 of yacc.c  */
#line 903 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 179:

/* Line 1455 of yacc.c  */
#line 905 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 180:

/* Line 1455 of yacc.c  */
#line 907 "pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 181:

/* Line 1455 of yacc.c  */
#line 910 "pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 182:

/* Line 1455 of yacc.c  */
#line 916 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 183:

/* Line 1455 of yacc.c  */
#line 918 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list;;}
    break;

  case 184:

/* Line 1455 of yacc.c  */
#line 923 "pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (4)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 185:

/* Line 1455 of yacc.c  */
#line 925 "pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 186:

/* Line 1455 of yacc.c  */
#line 927 "pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(1) - (1)].t_goal);;}
    break;

  case 187:

/* Line 1455 of yacc.c  */
#line 932 "pddl+.yacc"
    {(yyval.t_goal_list) = (yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 188:

/* Line 1455 of yacc.c  */
#line 934 "pddl+.yacc"
    {(yyval.t_goal_list) = new goal_list;;}
    break;

  case 189:

/* Line 1455 of yacc.c  */
#line 939 "pddl+.yacc"
    {(yyval.t_con_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 190:

/* Line 1455 of yacc.c  */
#line 941 "pddl+.yacc"
    {(yyval.t_con_goal) = new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);;}
    break;

  case 191:

/* Line 1455 of yacc.c  */
#line 944 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATEND,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 192:

/* Line 1455 of yacc.c  */
#line 946 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYS,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 193:

/* Line 1455 of yacc.c  */
#line 948 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIME,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 194:

/* Line 1455 of yacc.c  */
#line 950 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_WITHIN,(yyvsp[(4) - (5)].t_goal),NULL,(yyvsp[(3) - (5)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 195:

/* Line 1455 of yacc.c  */
#line 952 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATMOSTONCE,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 196:

/* Line 1455 of yacc.c  */
#line 954 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEAFTER,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 197:

/* Line 1455 of yacc.c  */
#line 956 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEBEFORE,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 198:

/* Line 1455 of yacc.c  */
#line 958 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYSWITHIN,(yyvsp[(5) - (6)].t_goal),(yyvsp[(4) - (6)].t_goal),(yyvsp[(3) - (6)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (6)].t_num_expression);;}
    break;

  case 199:

/* Line 1455 of yacc.c  */
#line 960 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDDURING,(yyvsp[(5) - (6)].t_goal),NULL,(yyvsp[(4) - (6)].t_num_expression)->double_value(),(yyvsp[(3) - (6)].t_num_expression)->double_value());delete (yyvsp[(3) - (6)].t_num_expression);delete (yyvsp[(4) - (6)].t_num_expression);;}
    break;

  case 200:

/* Line 1455 of yacc.c  */
#line 962 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDAFTER,(yyvsp[(4) - (5)].t_goal),NULL,0.0,(yyvsp[(3) - (5)].t_num_expression)->double_value());delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 201:

/* Line 1455 of yacc.c  */
#line 967 "pddl+.yacc"
    {(yyval.t_goal)= new simple_goal((yyvsp[(1) - (1)].t_proposition),E_POS);;}
    break;

  case 202:

/* Line 1455 of yacc.c  */
#line 969 "pddl+.yacc"
    {(yyval.t_goal)= new neg_goal((yyvsp[(3) - (4)].t_goal));simple_goal * s = dynamic_cast<simple_goal *>((yyvsp[(3) - (4)].t_goal));
       if(s && s->getProp()->head->getName()=="=") {requires(E_EQUALITY);} 
       else{requires(E_NEGATIVE_PRECONDITIONS);};;}
    break;

  case 203:

/* Line 1455 of yacc.c  */
#line 973 "pddl+.yacc"
    {(yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 204:

/* Line 1455 of yacc.c  */
#line 975 "pddl+.yacc"
    {(yyval.t_goal)= new disj_goal((yyvsp[(3) - (4)].t_goal_list));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 205:

/* Line 1455 of yacc.c  */
#line 978 "pddl+.yacc"
    {(yyval.t_goal)= new imply_goal((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_goal));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 206:

/* Line 1455 of yacc.c  */
#line 982 "pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal((yyvsp[(2) - (7)].t_quantifier),(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 207:

/* Line 1455 of yacc.c  */
#line 984 "pddl+.yacc"
    {(yyval.t_goal)= new comparison((yyvsp[(2) - (5)].t_comparison_op),(yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); 
        requires(E_FLUENTS);;}
    break;

  case 208:

/* Line 1455 of yacc.c  */
#line 990 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal));;}
    break;

  case 209:

/* Line 1455 of yacc.c  */
#line 992 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list;;}
    break;

  case 210:

/* Line 1455 of yacc.c  */
#line 997 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal));;}
    break;

  case 211:

/* Line 1455 of yacc.c  */
#line 999 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list;;}
    break;

  case 212:

/* Line 1455 of yacc.c  */
#line 1003 "pddl+.yacc"
    {(yyval.t_quantifier)=(yyvsp[(1) - (1)].t_quantifier);;}
    break;

  case 213:

/* Line 1455 of yacc.c  */
#line 1004 "pddl+.yacc"
    {(yyval.t_quantifier)=(yyvsp[(1) - (1)].t_quantifier);;}
    break;

  case 214:

/* Line 1455 of yacc.c  */
#line 1009 "pddl+.yacc"
    {(yyval.t_quantifier)=E_FORALL; 
        current_analysis->var_tab_stack.push(
        		current_analysis->buildForallTab());;}
    break;

  case 215:

/* Line 1455 of yacc.c  */
#line 1016 "pddl+.yacc"
    {(yyval.t_quantifier)=E_EXISTS;
        current_analysis->var_tab_stack.push(
        	current_analysis->buildExistsTab());;}
    break;

  case 216:

/* Line 1455 of yacc.c  */
#line 1023 "pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 217:

/* Line 1455 of yacc.c  */
#line 1028 "pddl+.yacc"
    {(yyval.t_proposition) = new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list));;}
    break;

  case 218:

/* Line 1455 of yacc.c  */
#line 1033 "pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 219:

/* Line 1455 of yacc.c  */
#line 1038 "pddl+.yacc"
    {(yyval.t_pred_decl_list)= (yyvsp[(3) - (4)].t_pred_decl_list);;}
    break;

  case 220:

/* Line 1455 of yacc.c  */
#line 1040 "pddl+.yacc"
    {yyerrok; (yyval.t_pred_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:predicates ...)");
	;}
    break;

  case 221:

/* Line 1455 of yacc.c  */
#line 1047 "pddl+.yacc"
    {(yyval.t_func_decl_list)= (yyvsp[(3) - (4)].t_func_decl_list);;}
    break;

  case 222:

/* Line 1455 of yacc.c  */
#line 1049 "pddl+.yacc"
    {yyerrok; (yyval.t_func_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:functions ...)");
	;}
    break;

  case 223:

/* Line 1455 of yacc.c  */
#line 1056 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 224:

/* Line 1455 of yacc.c  */
#line 1058 "pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 225:

/* Line 1455 of yacc.c  */
#line 1065 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 226:

/* Line 1455 of yacc.c  */
#line 1067 "pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 227:

/* Line 1455 of yacc.c  */
#line 1073 "pddl+.yacc"
    { (yyval.t_structure_store)=(yyvsp[(1) - (2)].t_structure_store); (yyval.t_structure_store)->push_back((yyvsp[(2) - (2)].t_structure_def)); ;}
    break;

  case 228:

/* Line 1455 of yacc.c  */
#line 1074 "pddl+.yacc"
    { (yyval.t_structure_store)= new structure_store; (yyval.t_structure_store)->push_back((yyvsp[(1) - (1)].t_structure_def)); ;}
    break;

  case 229:

/* Line 1455 of yacc.c  */
#line 1078 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_action_def); ;}
    break;

  case 230:

/* Line 1455 of yacc.c  */
#line 1079 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_event_def); requires(E_TIME); ;}
    break;

  case 231:

/* Line 1455 of yacc.c  */
#line 1080 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_process_def); requires(E_TIME); ;}
    break;

  case 232:

/* Line 1455 of yacc.c  */
#line 1081 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_durative_action_def); requires(E_DURATIVE_ACTIONS); ;}
    break;

  case 233:

/* Line 1455 of yacc.c  */
#line 1082 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_derivation_rule); requires(E_DERIVED_PREDICATES);;}
    break;

  case 234:

/* Line 1455 of yacc.c  */
#line 1086 "pddl+.yacc"
    {(yyval.t_dummy)= 0; 
    	current_analysis->var_tab_stack.push(
    					current_analysis->buildRuleTab());;}
    break;

  case 235:

/* Line 1455 of yacc.c  */
#line 1097 "pddl+.yacc"
    {(yyval.t_derivation_rule) = new derivation_rule((yyvsp[(3) - (5)].t_proposition),(yyvsp[(4) - (5)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 236:

/* Line 1455 of yacc.c  */
#line 1109 "pddl+.yacc"
    { (yyval.t_action_def)= current_analysis->buildAction(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
			(yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
			current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp); ;}
    break;

  case 237:

/* Line 1455 of yacc.c  */
#line 1113 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in action declaration.");
	 (yyval.t_action_def)= NULL; ;}
    break;

  case 238:

/* Line 1455 of yacc.c  */
#line 1126 "pddl+.yacc"
    {(yyval.t_event_def)= current_analysis->buildEvent(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		   (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
		   current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 239:

/* Line 1455 of yacc.c  */
#line 1131 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in event declaration.");
	 (yyval.t_event_def)= NULL; ;}
    break;

  case 240:

/* Line 1455 of yacc.c  */
#line 1143 "pddl+.yacc"
    {(yyval.t_process_def)= current_analysis->buildProcess(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		     (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
                     current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 241:

/* Line 1455 of yacc.c  */
#line 1147 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in process declaration.");
	 (yyval.t_process_def)= NULL; ;}
    break;

  case 242:

/* Line 1455 of yacc.c  */
#line 1159 "pddl+.yacc"
    { (yyval.t_durative_action_def)= (yyvsp[(10) - (11)].t_durative_action_def);
      (yyval.t_durative_action_def)->name= current_analysis->op_tab.symbol_put((yyvsp[(3) - (11)].cp));
      (yyval.t_durative_action_def)->symtab= current_analysis->var_tab_stack.pop();
      (yyval.t_durative_action_def)->parameters= (yyvsp[(6) - (11)].t_var_symbol_list);
      (yyval.t_durative_action_def)->dur_constraint= (yyvsp[(9) - (11)].t_goal); 
      delete [] (yyvsp[(3) - (11)].cp);
    ;}
    break;

  case 243:

/* Line 1455 of yacc.c  */
#line 1168 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in durative-action declaration.");
	 (yyval.t_durative_action_def)= NULL; ;}
    break;

  case 244:

/* Line 1455 of yacc.c  */
#line 1175 "pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->effects=(yyvsp[(3) - (3)].t_effect_lists);;}
    break;

  case 245:

/* Line 1455 of yacc.c  */
#line 1177 "pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->precondition=(yyvsp[(3) - (3)].t_goal);;}
    break;

  case 246:

/* Line 1455 of yacc.c  */
#line 1178 "pddl+.yacc"
    {(yyval.t_durative_action_def)= current_analysis->buildDurativeAction();;}
    break;

  case 247:

/* Line 1455 of yacc.c  */
#line 1183 "pddl+.yacc"
    { (yyval.t_goal)=(yyvsp[(1) - (1)].t_goal); ;}
    break;

  case 248:

/* Line 1455 of yacc.c  */
#line 1185 "pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 249:

/* Line 1455 of yacc.c  */
#line 1190 "pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 250:

/* Line 1455 of yacc.c  */
#line 1192 "pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 251:

/* Line 1455 of yacc.c  */
#line 1197 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_START);;}
    break;

  case 252:

/* Line 1455 of yacc.c  */
#line 1199 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_END);;}
    break;

  case 253:

/* Line 1455 of yacc.c  */
#line 1201 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_OVER_ALL);;}
    break;

  case 254:

/* Line 1455 of yacc.c  */
#line 1203 "pddl+.yacc"
    {timed_goal * tg = dynamic_cast<timed_goal *>((yyvsp[(4) - (5)].t_goal));
		(yyval.t_goal) = new timed_goal(new preference((yyvsp[(3) - (5)].cp),tg->clearGoal()),tg->getTime());
			delete tg;
			requires(E_PREFERENCES);;}
    break;

  case 255:

/* Line 1455 of yacc.c  */
#line 1208 "pddl+.yacc"
    {(yyval.t_goal) = new preference((yyvsp[(3) - (4)].t_goal));requires(E_PREFERENCES);;}
    break;

  case 256:

/* Line 1455 of yacc.c  */
#line 1212 "pddl+.yacc"
    {(yyval.t_dummy)= 0; current_analysis->var_tab_stack.push(
    				current_analysis->buildOpTab());;}
    break;

  case 257:

/* Line 1455 of yacc.c  */
#line 1217 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EQUALITY;;}
    break;

  case 258:

/* Line 1455 of yacc.c  */
#line 1218 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS;;}
    break;

  case 259:

/* Line 1455 of yacc.c  */
#line 1220 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TYPING;;}
    break;

  case 260:

/* Line 1455 of yacc.c  */
#line 1222 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_NEGATIVE_PRECONDITIONS;;}
    break;

  case 261:

/* Line 1455 of yacc.c  */
#line 1224 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DISJUNCTIVE_PRECONDS;;}
    break;

  case 262:

/* Line 1455 of yacc.c  */
#line 1225 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS;;}
    break;

  case 263:

/* Line 1455 of yacc.c  */
#line 1226 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_UNIV_PRECS;;}
    break;

  case 264:

/* Line 1455 of yacc.c  */
#line 1227 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_COND_EFFS;;}
    break;

  case 265:

/* Line 1455 of yacc.c  */
#line 1228 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_FLUENTS;;}
    break;

  case 266:

/* Line 1455 of yacc.c  */
#line 1230 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATIVE_ACTIONS;;}
    break;

  case 267:

/* Line 1455 of yacc.c  */
#line 1231 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TIME |
                      E_FLUENTS |
                      E_DURATIVE_ACTIONS; ;}
    break;

  case 268:

/* Line 1455 of yacc.c  */
#line 1235 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS |
		      E_TYPING | 
		      E_NEGATIVE_PRECONDITIONS |
		      E_DISJUNCTIVE_PRECONDS |
		      E_EQUALITY |
		      E_EXT_PRECS |
		      E_UNIV_PRECS |
		      E_COND_EFFS;;}
    break;

  case 269:

/* Line 1455 of yacc.c  */
#line 1244 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS |
		      E_UNIV_PRECS;;}
    break;

  case 270:

/* Line 1455 of yacc.c  */
#line 1248 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATION_INEQUALITIES;;}
    break;

  case 271:

/* Line 1455 of yacc.c  */
#line 1251 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_CONTINUOUS_EFFECTS;;}
    break;

  case 272:

/* Line 1455 of yacc.c  */
#line 1253 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_DERIVED_PREDICATES;;}
    break;

  case 273:

/* Line 1455 of yacc.c  */
#line 1255 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_TIMED_INITIAL_LITERALS;;}
    break;

  case 274:

/* Line 1455 of yacc.c  */
#line 1257 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_PREFERENCES;;}
    break;

  case 275:

/* Line 1455 of yacc.c  */
#line 1259 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_CONSTRAINTS;;}
    break;

  case 276:

/* Line 1455 of yacc.c  */
#line 1261 "pddl+.yacc"
    {log_error(E_WARNING,"Unrecognised requirements declaration ");
       (yyval.t_pddl_req_flag)= 0; delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 277:

/* Line 1455 of yacc.c  */
#line 1267 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 278:

/* Line 1455 of yacc.c  */
#line 1271 "pddl+.yacc"
    {(yyval.t_type_list)=(yyvsp[(3) - (4)].t_type_list); requires(E_TYPING);;}
    break;

  case 279:

/* Line 1455 of yacc.c  */
#line 1281 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(11) - (12)].t_problem); (yyval.t_problem)->name = (yyvsp[(5) - (12)].cp); (yyval.t_problem)->domain_name = (yyvsp[(9) - (12)].cp);;}
    break;

  case 280:

/* Line 1455 of yacc.c  */
#line 1283 "pddl+.yacc"
    {yyerrok; (yyval.t_problem)=NULL;
       	log_error(E_FATAL,"Syntax error in problem definition."); ;}
    break;

  case 281:

/* Line 1455 of yacc.c  */
#line 1289 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 282:

/* Line 1455 of yacc.c  */
#line 1290 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->objects= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 283:

/* Line 1455 of yacc.c  */
#line 1291 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->initial_state= (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 284:

/* Line 1455 of yacc.c  */
#line 1292 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->the_goal= (yyvsp[(1) - (2)].t_goal);;}
    break;

  case 285:

/* Line 1455 of yacc.c  */
#line 1294 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 286:

/* Line 1455 of yacc.c  */
#line 1295 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->metric= (yyvsp[(1) - (2)].t_metric);;}
    break;

  case 287:

/* Line 1455 of yacc.c  */
#line 1296 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->length= (yyvsp[(1) - (2)].t_length_spec);;}
    break;

  case 288:

/* Line 1455 of yacc.c  */
#line 1297 "pddl+.yacc"
    {(yyval.t_problem)=new problem;;}
    break;

  case 289:

/* Line 1455 of yacc.c  */
#line 1300 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 290:

/* Line 1455 of yacc.c  */
#line 1303 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 291:

/* Line 1455 of yacc.c  */
#line 1306 "pddl+.yacc"
    {(yyval.vtab) = current_analysis->buildOpTab();;}
    break;

  case 292:

/* Line 1455 of yacc.c  */
#line 1309 "pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(3) - (4)].t_goal);delete (yyvsp[(2) - (4)].vtab);;}
    break;

  case 293:

/* Line 1455 of yacc.c  */
#line 1314 "pddl+.yacc"
    { (yyval.t_metric)= new metric_spec((yyvsp[(3) - (5)].t_optimization),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 294:

/* Line 1455 of yacc.c  */
#line 1316 "pddl+.yacc"
    {yyerrok; 
        log_error(E_FATAL,"Syntax error in metric declaration.");
        (yyval.t_metric)= NULL; ;}
    break;

  case 295:

/* Line 1455 of yacc.c  */
#line 1323 "pddl+.yacc"
    {(yyval.t_length_spec)= new length_spec(E_BOTH,(yyvsp[(4) - (7)].ival),(yyvsp[(6) - (7)].ival));;}
    break;

  case 296:

/* Line 1455 of yacc.c  */
#line 1326 "pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_SERIAL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 297:

/* Line 1455 of yacc.c  */
#line 1330 "pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_PARALLEL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 298:

/* Line 1455 of yacc.c  */
#line 1336 "pddl+.yacc"
    {(yyval.t_optimization)= E_MINIMIZE;;}
    break;

  case 299:

/* Line 1455 of yacc.c  */
#line 1337 "pddl+.yacc"
    {(yyval.t_optimization)= E_MAXIMIZE;;}
    break;

  case 300:

/* Line 1455 of yacc.c  */
#line 1342 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(2) - (3)].t_expression);;}
    break;

  case 301:

/* Line 1455 of yacc.c  */
#line 1343 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term);;}
    break;

  case 302:

/* Line 1455 of yacc.c  */
#line 1344 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_num_expression);;}
    break;

  case 303:

/* Line 1455 of yacc.c  */
#line 1345 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 304:

/* Line 1455 of yacc.c  */
#line 1347 "pddl+.yacc"
    {(yyval.t_expression) = new violation_term((yyvsp[(3) - (4)].cp));;}
    break;

  case 305:

/* Line 1455 of yacc.c  */
#line 1348 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 306:

/* Line 1455 of yacc.c  */
#line 1352 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 307:

/* Line 1455 of yacc.c  */
#line 1353 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 308:

/* Line 1455 of yacc.c  */
#line 1354 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 309:

/* Line 1455 of yacc.c  */
#line 1355 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 310:

/* Line 1455 of yacc.c  */
#line 1359 "pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 311:

/* Line 1455 of yacc.c  */
#line 1361 "pddl+.yacc"
    {(yyval.t_expression) = new plus_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 312:

/* Line 1455 of yacc.c  */
#line 1365 "pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 313:

/* Line 1455 of yacc.c  */
#line 1367 "pddl+.yacc"
    {(yyval.t_expression) = new mul_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 314:

/* Line 1455 of yacc.c  */
#line 1373 "pddl+.yacc"
    {(yyval.t_plan)= (yyvsp[(2) - (2)].t_plan); 
         (yyval.t_plan)->push_front((yyvsp[(1) - (2)].t_step)); ;}
    break;

  case 315:

/* Line 1455 of yacc.c  */
#line 1376 "pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].fval));;}
    break;

  case 316:

/* Line 1455 of yacc.c  */
#line 1378 "pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].ival));;}
    break;

  case 317:

/* Line 1455 of yacc.c  */
#line 1380 "pddl+.yacc"
    {(yyval.t_plan)= new plan;;}
    break;

  case 318:

/* Line 1455 of yacc.c  */
#line 1385 "pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(3) - (3)].t_step); 
         (yyval.t_step)->start_time_given=1; 
         (yyval.t_step)->start_time=(yyvsp[(1) - (3)].fval);;}
    break;

  case 319:

/* Line 1455 of yacc.c  */
#line 1389 "pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(1) - (1)].t_step);
	 (yyval.t_step)->start_time_given=0;;}
    break;

  case 320:

/* Line 1455 of yacc.c  */
#line 1395 "pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (4)].t_step); 
	 (yyval.t_step)->duration_given=1;
         (yyval.t_step)->duration= (yyvsp[(3) - (4)].fval);;}
    break;

  case 321:

/* Line 1455 of yacc.c  */
#line 1399 "pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (1)].t_step);
         (yyval.t_step)->duration_given=0;;}
    break;

  case 322:

/* Line 1455 of yacc.c  */
#line 1405 "pddl+.yacc"
    {(yyval.t_step)= new plan_step( 
              current_analysis->op_tab.symbol_get((yyvsp[(2) - (4)].cp)), 
	      (yyvsp[(3) - (4)].t_const_symbol_list)); delete [] (yyvsp[(2) - (4)].cp);
      ;}
    break;

  case 323:

/* Line 1455 of yacc.c  */
#line 1412 "pddl+.yacc"
    {(yyval.fval)= (yyvsp[(1) - (1)].fval);;}
    break;

  case 324:

/* Line 1455 of yacc.c  */
#line 1413 "pddl+.yacc"
    {(yyval.fval)= (float) (yyvsp[(1) - (1)].ival);;}
    break;



/* Line 1455 of yacc.c  */
#line 4700 "pddl+.cpp"
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
      /* If just tried and failed to reuse lookahead token after an
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

  /* Else will try to reuse lookahead token after shifting the error
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

#if !defined(yyoverflow) || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
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



/* Line 1675 of yacc.c  */
#line 1416 "pddl+.yacc"


#include <cstdio>
#include <iostream>
int line_no= 1;
using std::istream;
#include "lex.yy.cc"

namespace VAL {
extern yyFlexLexer* yfl;
};


int yyerror(char * s)
{
    return 0;
}

int yylex()
{
    return yfl->yylex();
}

