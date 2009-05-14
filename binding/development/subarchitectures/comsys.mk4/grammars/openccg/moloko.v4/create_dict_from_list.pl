# This tool creates the dict.xsl file by first converting
# the "user-friendly" dict.list into the appropriate xml format,
# and then appends this to the base grammar in dict template.xsl


$tab = "  ";
$base_input_file = "dict-base.xsl";
$append_input_file = "dict.list";
$output_file = "dict.xsl";

# Read in the base grammar as defined in grammar template
open (FILE_IN, '<', $base_input_file) || die "Can't read '$input_file': $!";
  $grammar = join ('', <FILE_IN>);
close FILE_IN;

# Create dictionary from the file dict.list and append it
# to the grammar

open(FILE_IN, $append_input_file) or die "Error opening file dict.template";
  build_dict_from_list();
close FILE_IN;  
  
# Append the dict-list to the base
$grammar =~ s/<!-- __INSERT__HERE__ -->/$dictlist/gi;

# Write the grammar out
open (FILE_OUT, '>', $output_file) || die "Can't write '$output_file': $!";
   print FILE_OUT $grammar;
close FILE_OUT;




##########################################################################################
# SUBROUTINES
##########################################################################################

sub build_dict_from_list{


# Run through the dict.list file

$dictlist="";  # This will be the concantenated string of all
			   # entries included in dict.list

  while ( <FILE_IN> ) {
   chomp;
   if(/^----pos (.*)/){
      $pos = $1;
	  $dictlist .= "<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ \n".  
            "     +  ".$pos."\n".
            "     ++++++++++++++++++++++++++++++++++++++++++++++++ -->\n\n\n";
   }  # Pos read
   elsif(/^--class (.*)/){$class = $1;} # class (type read)
   elsif(/^::/ or $_ eq ""){} # comment or blank space
   else{                  #everything else is an entry
	  
	  #empty the lists being used
      @word_forms = ();
	  @macros = (); 
      @families = ();
	  	
      # this handles alternate forms of the entry (enclosed in {})
	 
	  @entries = split(/:/, $_);            #split multiple entries up
      ($pred)  = split(/ /, $entries[0]);   #stem is always 1st word
	  
	  $stem = $pred."__".$class;
	  #$pred = $stem;              #pred value is set to stem, stem may change
	  
	  # add the entry then skip a space
	  add_entry();
	  $dictlist .= "\n";
	}
  }
}

#######################################################################################

sub add_entry{

	# Based on POS, handle differently #
	# Each case must 1) split up all the entries (sep'd by : in .list)
	#                2) for each, split and handle accordingly
	#                3) build list of word_forms
	#                4) set stem (if dif), set famlilies, and print 
	
    # Adjectives have 3 forms, basic, comparative and superlative
	    if ($pos eq "ADJ"){
		  foreach $entry (@entries) {
		   
			 @degrees = split(" ", $entry);
			 push (@word_forms, [@degrees[0], "\@degree.base"]);
		     push (@word_forms, [@degrees[1], "\@degree.comparative"]);
		     push (@word_forms, [@degrees[2], "\@degree.superlative"]);
		  }
			  
		  @families=("adj.property");	
		  print_entry();
    
	
	# There are many kinds of nouns: bare, mass, and standard sg/pl
	}elsif($pos eq "N"){
			
		
		
		# Mass nouns
		if (substr($pred,length($pred)-5,5) eq "-mass"){
		   $pred = substr($pred,0,length($pred)-5);
		   @entries[0]=$pred;

		   foreach $entry (@entries) {push (@word_forms, [$entry, "\@pers.3rd"]);}

		   @families=("noun.mass");
           print_entry();
		   
		# Bare-Nps for things like Names   
	    } elsif (substr($pred,length($pred)-5,5) eq "-bare"){ 

           $pred = substr($pred,0,length($pred)-5);
     	   @entries[0]=$pred;		   
	
		   foreach $entry (@entries) {push (@word_forms, [$entry, "\@pers.3rd"]);}
		   
		   @families=("bare-np");
           print_entry();
       
       ### NORMAL NOUN WITH SINGULAR AND PLURAL FORM
	   } else {    
		
        @singular_forms = ();
		@plural_forms = (); 
	
        foreach $entry (@entries) {
				
		    if ($entry =~ /^(.*) (.*)/){
		       $sg = $1;
			   $pl = $2;
		    }else{
		       $sg = $entry;
			   $pl = $entry."s";
			}
			
	        push (@singular_forms, [$sg, "\@pers.3rd"]);
	        push (@plural_forms,   [$pl, "\@pers.3rd"]);
	    }
		
		# add all the single forms 
		$stem = $pred."-sg";
		@families=("noun");
		@word_forms = @singular_forms;	
    	print_entry();

        # add all the plural forms in a separate entry 
	    $stem = $pred."-pl";
		@word_forms = @plural_forms;
		@families=("noun.plural");	
    	print_entry();
	 }    
	
	
	# advs have a family of type adv.class (ex. adv.direction)
    }elsif($pos eq "ADV"){
	
    	foreach $entry (@entries) {push (@word_forms, [$entry, ""]);}
		if ($class eq "mod-adj"){
		 @families=("adv-mod-adj");	 
		}else{
		  $adv_type = substr $class, 2;  #strip of m-
		  @families=("adv.".$adv_type);	
		}
		
		print_entry();
	
	# preps have default mod n and mod s versions, unless specified 
	# otherwise in dict.list
			
	}elsif($pos eq "PREP"){
	 
		@forms = ();
	    
     	@forms = split(/ /, @entries[0]);
        if(@forms != 1){
	      push(@families, @forms[1]);
	    }else{
	      push(@families, "n");
	      push(@families, "s");
	    }
		
		#handle cases where it is "of-np" i.e. right of, on top of, etc
		$of_flag = "";
		
		if (substr($pred,length($pred)-3,3) eq "-of"){
		   $pred = substr($pred,0,length($pred)-3);
		   $of_flag = "of-np.";
		}
		
		# set up word-forms
		@entries[0] = $pred;
		foreach $entry (@entries) {push(@word_forms, [$entry, ""])};
		
        $mod_type = substr $class, 2;
		foreach $family(@families){
			$family = "prep-mod-".$family.".".$of_flag.$mod_type;
		}
		print_entry();
 
    # dependent clause markers such as if, when, because, etc.
	
    }elsif($pos eq "MARKER"){
	
    	foreach $entry (@entries) {push (@word_forms, [$entry, ""]);}
		
		$clause_type = substr $class, 2;
		@families=("dep-clause-marker.".$clause_type);
		print_entry();
		
    
	# Verbs have different vforms (nonfinite, past, etc) and
	# specify their families explicitly (--> tv iv, etc)
	
	}elsif($pos eq "V"){
	
		@forms = ();
		@ind_forms = (); 
		@imp_forms = ();
		
		
		# take out -> and families from 1st entry (could be changed to last
	    # so it looks x:x:x: -> familes...maybe better...
		
		@entries[0] =~ /(.*) -> (.*)/;		
		@entries[0] = $1;		
		@families = split(/ /, $2);
								
    	foreach $entry (@entries) {
		  		   
	       @forms = split(/ /, $entry);
		   if (@forms == 1){
			 @forms[1] = @forms[0]."s";
			 @forms[2] = @forms[0]."ing";
			 @forms[3] = @forms[0]."ed";
		   }
		 
	       push (@ind_forms, [@forms[0], "\@nonfin"]);
		   push (@ind_forms, [@forms[0], "\@finite \@pres \@imp \@pers.3rd-agr"]); # THIS FOR DIRECTED VOCATIVE IMPERATIVES Robot pick that up
		   push (@ind_forms, [@forms[2], "\@vform.progr"]);
		   push (@ind_forms, [@forms[0], "\@finite \@pres \@ind \@pers.non-3rd-agr \@num.sg-agr "]);
           push (@ind_forms, [@forms[0], "\@finite \@pres \@ind \@num.pl-agr "]);
	   	   push (@ind_forms, [@forms[1], "\@finite \@pres \@ind \@pers.3rd-agr \@num.sg-agr"]);
		   push (@ind_forms, [@forms[3], "\@finite \@past \@ind"]);
	
	# PRE-VERB AGREEMENT		   		   
	#	   push (@ind_forms, [@forms[0], "\@nonfin"]);
     #      push (@ind_forms, [@forms[0], "\@finite \@pres \@ind"]);
	#	   push (@ind_forms, [@forms[1], "\@finite \@pres \@ind"]);
	#	   push (@ind_forms, [@forms[2], "\@vform.progr"]);
	#	   push (@ind_forms, [@forms[3], "\@finite \@past \@ind"]);
		   
  
  
		# This and IMP* section below were taken out for a long while becuase I was
		# trying to handle imperatives via lexical rule (see unary rules) but it wasnt working prop
		push (@imp_forms, [@forms[0], "\@finite \@pres \@imp-addressee"]);
           
	    }  
		  		
		# add the indicative entry 
		$stem = $pred."__".$class;
		@familiescopy = @families;
		#print "????", $family, "\n"; 
		@word_forms = @ind_forms;	
    	print_entry();

        		   # IMP *
        # add the imperitive entry 
		$stem = $pred."-NO-SUBJ__".$class;
		@families = @familiescopy;		
		foreach $family (@families){$family .= ".NO-SUBJ";}
		@word_forms = @imp_forms;	
     	print_entry();

    }elsif($pos eq "WORD"){
	    
		foreach $entry (@entries) {push (@word_forms, [$entry, ""]);}
		$stem = $pred; # don't want wform to be X__blah	
      	print_entry();
		for ($i = 1; $i <= 3; $i++){
		   $index = 50 + $i;
		   
		  # This is with featvar WORD
		  # $dictlist .= "<macro name=\"\@mwe". $i . "-". $pred . "\"> <fs id=\"" . $index."\">"
		  #               ."<feat attr=\"wform\"> <featvar name=\"WORD".$i .":". $pred."\"/></feat></fs></macro>\n";

          $dictlist .= "<macro name=\"\@mwe". $i . "-". $pred . "\"> <fs id=\"" . $index."\">"
		                 ."<feat attr=\"wform\" val=\"".$pred."\"/> </fs> </macro>\n";

	    }
    }
}

#######################################################################################

sub print_entry{
    $dictlist .= header().forms().members().footer();
}

sub header{
    return "<entry pred=\"".$pred."\" stem=\"".$stem."\"  pos=\"".$pos."\" class=\"".$class."\">\n"
}

sub footer{
    return "</entry>"."\n"
}

sub forms{

	$form_string = "";
	foreach $form (@word_forms){

	    $form_string .= $tab."<word form=\"".@{$form}[0]."\"";  #print form
		if (@{$form}[1] ne ""){
		  $form_string .= " macros=\"".@{$form}[1]."\"";  #add macros if there are any 
		}
		$form_string .= "/>\n"; # close word form line
   }
	   
    return $form_string;
}

sub members{
	$family_string = "";
    foreach $mem (@families){
       $family_string .=  $tab."<member-of family=\"".$mem."\"/>"."\n"
	}
	return $family_string;
}