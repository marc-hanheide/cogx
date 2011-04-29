#!/usr/bin/env perl

use warnings;
use strict;

if ($#ARGV != 2) {
	print "Usage: generate-correlations.pl P-MATCH P-NOMATCH P-UNK < DOMAIN-FILE\n";
	exit 1;
}

my $p_match = $ARGV[0];
my $p_nomatch = $ARGV[1];
my $p_unk = $ARGV[2];

#my $p_distrib = 1.0 - $p_match - $p_unk;
#
#if ($p_distrib < 0.0) {
#	die "remaining probability mass lesser than zero";
#}

my $current_feat = "";
my %val_to_feat = ();
my %feat_counts = ();

while (<STDIN>) {
	chomp;
	my $line = $_;
	$line =~ s/#.*$//;

	if ($line =~ /\s*([a-zA-Z]+):\s*$/) {
		my $feat = $1;
		$current_feat = $feat;
		if (!defined($feat_counts{$current_feat})) {
			$feat_counts{$current_feat} = 0;
		}
	}

	if ($line =~ /\s+([a-zA-Z]+)\s*$/) {
		my $val = $1;
		$current_feat ne "" or die "current_feature unset";
		$feat_counts{$current_feat}++;
		$val_to_feat{$val} = $current_feat;
	}
}

my @cs = ();

for my $i (keys(%val_to_feat)) {
	my $f_i = $val_to_feat{$i};

	push @cs, "(ling_to_observed($val_to_feat{$i}($i), unknown)) = p($p_unk)";

	for my $j (keys(%val_to_feat)) {
		my $f_j = $val_to_feat{$j};

#		push @cs, "(ling_to_observed(unknown, $val_to_feat{$j}($j))) = p($p_unk)";

		my $p = 0.0;

		if ($i eq $j) {
			$p = $p_match;
		}
		else {
			$p = $p_nomatch;
#			if ($feat_counts{$f_i} eq $feat_counts{$f_j}) {
#				$p = (1.0 - $p_match) / $feat_counts{$f_i};
#			}
		}

		if ($p > 0.0) {
			push @cs, "(ling_to_observed($val_to_feat{$i}($i), $val_to_feat{$j}($j))) = p($p)";
		}
	}
}

#print "correlation = [\n\t"
print "\t" . join(",\n\t", @cs) . "\n].\n";
