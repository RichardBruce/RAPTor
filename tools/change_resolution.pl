#!/usr/bin/perl

use strict;
use Getopt::Long;

my ($help, $file, $format, $x_res, $y_res, $out);
                
GetOptions (
    "help"      => \$help,
    "file:s"    => \$file,
    "x_res:i"   => \$x_res,
    "y_res:i"   => \$y_res,
    "format:s"  => \$format
    );

# Check arguements
if ($help or !$file or !$format or !$x_res or !$y_res)
{
    print "Usage fix_absoloute_paths -file <file> -format <format> -x_res <x resolution> -y_res <y resolution>\n";
    print "   -file     : Specify the input and output file name\n";
    print "   -format   : Specify the file format, current supported file formats are lwo\n";
    print "   -x_res    : The x resolution required\n";
    print "   -x_res    : The y resolution required\n";
    print "\n";
    exit;
}

# Only nff files contain resolution information
($file ne "nff") or die "Error: Only nff files contain resolution information\n";

# Open file for amend
open(FILE, "+< $file") or die "Error: Can't read $file: $!";

# Amend
$out = '';
while (<FILE>)
{
    s/resolution\s+\d+\s+\d+/resolution $x_res $y_res/;
    $out .= $_;
}
seek(FILE, 0, 0)            or die "Error: Can't seek to start of $file: $!";
print FILE $out             or die "Error: Can't print to $file: $!";
truncate(FILE, tell(FILE))  or die "Error: Can't truncate $file: $!";
close(FILE)                 or die "Error: Can't close $file: $!";

exit 0;
