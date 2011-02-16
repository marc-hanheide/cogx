#!/usr/bin/env perl

use warnings;
use strict;

if ($#ARGV != 1) {
	print "Usage: mln-to-abd.pl DOMAIN-MLN CORREL-MLN > RULEFILE\n";
	exit 1
}

my $domain_filename = $ARGV[0];
my $correl_filename = $ARGV[1];

my %rev_grpfeat = ();
my %rev_modfeat = ();

print STDERR "reading the domain definition...\n";
open DF, "$domain_filename" or die "I/O error while reading $domain_filename";
while (<DF>) {
	chomp;
	my $line = $_;
	if ($line =~ /grpfeat\(([A-Za-z0-9]+),\s*([A-Za-z0-9]+).*\)\./) {
		$rev_grpfeat{$2} = $1;
	}
	if ($line =~ /modfeat\(([A-Za-z0-9]+),\s*([A-Za-z0-9]+).*\)\./) {
		$rev_modfeat{$2} = $1;
	}
}
close DF;

my %conversion = ();

foreach my $k (keys %rev_grpfeat) {
	my $mod = $rev_modfeat{$k};
	my $grp = $rev_grpfeat{$k};

	my $pred = $grp;
	my $arg = $k;

	if ($mod eq "Vision") {
		if ($pred =~ /^V([A-Za-z]+)[0-9]*$/) {
			$pred = $1;
		}
		if ($arg =~ /^V([A-Za-z]+)[0-9]*$/) {
			$arg = $1;
		}
	}
	elsif ($mod eq "Language") {
		if ($pred =~ /^L([A-Za-z]+)[0-9]*$/) {
			$pred = $1;
		}
		if ($pred eq "Type") {
			$pred = "ObjectType";
		}
		if ($arg =~ /^L([A-Za-z]+)[0-9]*$/) {
			$arg = $1;
		}
	}
	elsif ($mod eq "Affordance") {
		if ($arg =~ /^A([A-Za-z]+)[0-9]*$/) {
			$arg = $1;
		}
	}
	$pred = lc($pred);
	$arg = lc($arg);

	$conversion{$k} = "$pred($arg)";
}

#foreach my $k (keys %conversion) {
#	print "$k -> $conversion{$k}\n";
#}

my @assumables = ();

print STDERR "converting correlations...\n";
open CF, "$correl_filename" or die "I/O error while reading $correl_filename";
while (<CF>) {
	chomp;
	my $line = $_;
	if ($line =~ /^(-?[0-9]+\.[0-9]+)\s+(.*)$/) {
		my $weight = $1;
		my $rule = $2;
		my @literals = split /\s+v\s+/, $rule;
		if ($#literals == 3) {
			if ($literals[0] =~ /^!/) {
				# segregative rule
			}
			else {
				# agregative rule
				my ($head, $part, $ling, $obs) = @literals;
				my $left;
				my $right;

				if ($ling =~ /^!proxyfeat\([A-Za-z0-9]+,\s*([A-Za-z0-9]+)\)$/) {
					$left = $1;
				}
				if ($obs =~ /^!proxyfeat\([A-Za-z0-9]+,\s*([A-Za-z0-9]+)\)$/) {
					$right = $1;
				}

				my $prob = exp($weight) / (1.0 + exp($weight));

				push @assumables, "(ling_to_observed($conversion{$left}, $conversion{$right})) = p($prob)";
			}
		}
	}
}
close DF;

print "correlation = [\n\t" . join(",\n\t", @assumables) . "\n].\n";
