%code requires { /*** C/C++ Declarations ***/

#include <memory>
#include <string>
#include <vector>

#include "commands.h"

}

/*** yacc/bison Declarations ***/

/* Require bison 3.2 or later, which stores the semantic values as variants */
%require "3.2"

/* add debug output code to generated parser. disable this for release
 * versions. */
/*%debug*/

/* start symbol is named "start" */
%start start

/* write out a header file containing the token defines */
%defines

/* use the C++ skeleton file */
%language "c++"

/* namespace to enclose parser in */
%define api.namespace {SlamParser}

/* set the parser's class identifier */
%define api.parser.class {Parser}

/* the semantic values own what they hold, so that the parser frees the values
 * it discards on errors */
%define api.value.type variant

/* keep track of the current position within the input */
%locations
%initial-action
{
    // initialize the initial location object
    @$.begin.filename = @$.end.filename = &driver.streamname;
};

/* The driver is passed by reference to the parser and to the scanner. This
 * provides a simple but effective pure interface, not relying on global
 * variables. */
%parse-param { class Driver& driver }

/* verbose error messages */
%define parse.error verbose

 /*** BEGIN TOKEN - Change the example grammar's tokens below ***/

%token			END	     0	"end of file"
%token			EOL		"end of line"
%token <int>		INTEGER		"integer"
%token <double>		DOUBLE		"double"
%token <std::string>	STRING		"string"
%token			ADD		"ADD"
%token <std::string>	V_SE2		"Vertex SE2"
%token <std::string>	V_SE3		"Vertex SE3"
%token <std::string>	E_SE2		"Edge SE2"
%token <std::string>	E_SE3		"Edge SE3"
%token			FIX		"Fix"
%token			SOLVE_STATE	"Solve State"
%token			QUERY_STATE	"Query State"

%type <double>	NUMBER
%type <std::unique_ptr<CommandNode>>	query_state solve_state fix_node add_se2 add_se3 command
%type <std::vector<int>>	int_list

 /*** END TOKEN - Change the example grammar's tokens above ***/

%{

#include "driver.h"
#include "scanner.h"
#include "slam_context.h"

/* this "connects" the bison parser in the driver to the flex scanner class
 * object. it defines the yylex() function call to pull the next token from the
 * current lexer object of the driver context. */
#undef yylex
#define yylex driver.lexer->lex

%}

%% /*** Grammar Rules ***/

 /*** BEGIN GRAMMAR ***/

int_list : INTEGER
           {
              $$.push_back($1);
           }
         | int_list INTEGER
           {
             $1.push_back($2);
             $$ = std::move($1);
           }

NUMBER : INTEGER
         {
	   $$ = $1;
	 }
       | DOUBLE
         {
	   $$ = $1;
	 }

add_se2 : ADD V_SE2 INTEGER
           {
	       $$ = std::make_unique<AddNode>($2, $3, 3);
	   }
         | ADD V_SE2 INTEGER NUMBER NUMBER NUMBER
           {
              std::vector<double> values;
              values.push_back($4);
              values.push_back($5);
              values.push_back($6);
	      $$ = std::make_unique<AddNode>($2, $3, 3, values);
	   }
         | ADD E_SE2 INTEGER INTEGER INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER   
           {
              std::vector<double> values;
              values.push_back($6);
              values.push_back($7);
              values.push_back($8);
              std::vector<double> information;
              information.push_back($9);
              information.push_back($10);
              information.push_back($11);
              information.push_back($12);
              information.push_back($13);
              information.push_back($14);
              $$ = std::make_unique<AddEdge>($2, $3, values.size(), $4, $5, values, information);
	   }

add_se3 : ADD V_SE3 INTEGER
           {
               $$ = std::make_unique<AddNode>($2, $3, 6);
	   }
         | ADD V_SE3 INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
           {
              std::vector<double> values{$4, $5, $6, $7, $8, $9};
	      $$ = std::make_unique<AddNode>($2, $3, 6, values);
	   }
         | ADD E_SE3 INTEGER INTEGER INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
           {
              std::vector<double> values;
              values.push_back($6);
              values.push_back($7);
              values.push_back($8);
              values.push_back($9);
              values.push_back($10);
              values.push_back($11);
              std::vector<double> information;
              information.push_back($12);
              information.push_back($13);
              information.push_back($14);
              information.push_back($15);
              information.push_back($16);
              information.push_back($17);
              information.push_back($18);
              information.push_back($19);
              information.push_back($20);
              information.push_back($21);
              information.push_back($22);
              information.push_back($23);
              information.push_back($24);
              information.push_back($25);
              information.push_back($26);
              information.push_back($27);
              information.push_back($28);
              information.push_back($29);
              information.push_back($30);
              information.push_back($31);
              information.push_back($32);
              $$ = std::make_unique<AddEdge>($2, $3, values.size(), $4, $5, values, information);
	   }

fix_node : FIX INTEGER
           {
              std::vector<int> values;
              values.push_back($2);
              $$ = std::make_unique<FixNode>("FIX", values);
           }

solve_state : SOLVE_STATE
            {
              $$ = std::make_unique<SolveSate>("SOLVE_STATE");
            }

query_state : QUERY_STATE 
            {
              $$ = std::make_unique<QueryState>("QUERY_STATE");
            }
            | QUERY_STATE int_list
            {
              $$ = std::make_unique<QueryState>("QUERY_STATE", $2);
            }

command : add_se2
        {
          $$ = std::move($1);
        }
        | add_se3
        {
          $$ = std::move($1);
        }
        | fix_node
        {
          $$ = std::move($1);
        }
        | solve_state
        {
          $$ = std::move($1);
        }
        | query_state
        {
          $$ = std::move($1);
        }

start	: /* empty */
        | start EOL
        | start command ';'
          {
            driver.slamContext.process($2.get());
	  }

 /*** END GRAMMAR ***/

%% /*** Additional Code ***/

void SlamParser::Parser::error(const Parser::location_type& l, const std::string& m)
{
  driver.error(l, m);
}
