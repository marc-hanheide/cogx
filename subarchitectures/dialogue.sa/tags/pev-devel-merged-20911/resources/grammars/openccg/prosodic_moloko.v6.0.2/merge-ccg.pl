# This script combines all .ccg files within this directory into 
# a single .ccg file

$dir = shift;
$outfile = shift;

print "\nmerging all .ccg files in directory ". $dir ." into file ".$outfile."\n\n";

opendir(DIR, $dir) or die;
open(FILE_OUT, '>', $outfile);

# search the directory for all .ccg files
# read through each, writing its contents to FILE_OUT

print FILE_OUT 
"\n \n \n ################################################################ 
# I AM A GENERATED FILE (by merge-ccg.pl)                      #  
# DO NOT EDIT ME DIRECTLY !!!                                  #
# INSTEAD, EDIT THE SOURCE .CCG FILES IN ./ccg-files FOLDER    #
################################################################ \n \n ";

foreach $file (grep /\.ccg$/i, sort(readdir(DIR))) {   

	open (FILE, $dir.$file);    
	print "    ".$file."\n";
	while ( <FILE> ) {
		print FILE_OUT $_;
	}
    print FILE_OUT "\n";
}

print "\nfiles merged\n\n";

close (FILE);
closedir (DIR);
