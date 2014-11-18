#!/usr/bin/perl
use strict;
use Cwd;

# Check usage
($#ARGV != 2)  or die "Two arguments expect " . $#ARGV . " given\n";


# Create output dir if needed
my $dir = cwd;
my $project_name;
if ($dir =~ m/(\w+)$/)
{
	$project_name = $1;
}
else
{
    die "Error: Cant determine project name\n";
}

my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) = localtime();
my $date =  ($year + 1900) . "-" . ($mon + 1) . "-" . $mday;
my $output_dir = $ARGV[1] . $project_name . "/" .  $date;
`mkdir -p $output_dir`;


# Amend the index with a link to todays data
my $project_index = $ARGV[1] . $project_name . "/index.html";
my $new_project_index = $ARGV[1] . $project_name . "/index1.html";
open (INDEX_T, "<", $project_index) or die "Error: Can't find project index.html";
open (INDEX_F, ">", $new_project_index) or die "Error: Can't open new project index.html";
while (my $line = <INDEX_T>)
{
    chomp;

    # Search for the historic section
    if ($line =~ m/<h2>Historic<\/h2>/)
    {
        print INDEX_F $line;

        # Check if todays date was already add and if not add it
        $line = <INDEX_T>;
        if ($line !~ m/$date/)
        {
            print INDEX_F "    <p><a href=\"./$date\">$date</a></p>\n";
        }
    }

    print INDEX_F $line;
}
close (INDEX_T);
close (INDEX_F);

`mv  $new_project_index $project_index`;


# Open output file
open (OUTFILE, ">", $output_dir . "/index.html") or die "Error: Can't open output file";

# Output html header
print OUTFILE "<!DOCTYPE html>\n";
print OUTFILE "<html>\n";

print OUTFILE "<head>\n";
print OUTFILE "<style>\n";
print OUTFILE "table, th, td {\n";
print OUTFILE "    border: 1px solid black;\n";
print OUTFILE "    border-collapse: collapse;\n";
print OUTFILE "}\n";
print OUTFILE "th, td {\n";
print OUTFILE "    padding: 5px;\n";
print OUTFILE "}\n";
print OUTFILE "</style>\n";
print OUTFILE "</head>\n";

print OUTFILE "<body>\n";
print OUTFILE "<table style=\"width:100%\">\n";
print OUTFILE "    <tr>\n";
print OUTFILE "        <th>Scene</th>\n";
print OUTFILE "        <th># Primitives</th>\n";
print OUTFILE "        <th># Lights</th>\n";
print OUTFILE "        <th>Parser Time ms</th>\n";
print OUTFILE "        <th>Render Time ms</th>\n";
print OUTFILE "    </tr>\n";

my $scene       = "No Scenes Found";
my $primitives  = "--";
my $lights      = "--";
my $parser_time = "--";
my $render_time = "--";

# Open input file and parse
open (LOGFILE, "<", $ARGV[0]) or die "Error: Can't find input file";
while (<LOGFILE>)
{
    chomp;
    if (m/Opening\s+input\s+file:\s+.*\/(\w+\/\w+\/\w+\.\w+)/)
    {
        # Output last scene
        if ($scene ne "No Scenes Found")
        {
            output_scene();
        }

        # Graph new scene name
        $scene = $1;

        # Reset data
        $primitives     = "--";
        $lights         = "--";
        $parser_time    = "--";
        $render_time    = "--";
    }
    elsif (m/Rendering:\s+(\d+)\s+primitives,\s+of\s+which\s+lights:\s+(\d+)/)
    {
        # Get primitive counts
        $primitives = $1;
        $lights     = $2;
    }
    elsif (m/Parser\s+took:\s+(\d)+ms/)
    {
        # Get parser run time
        $parser_time = $1;
    }
    elsif (m/Test\s+took:\s+(\d+)ms/)
    {
        # Get rendering run time
        $render_time = $1;
    }
}

# Output the last scene
output_scene();

# Output html footer
print OUTFILE "</table>\n";

print OUTFILE "</body>\n";
print OUTFILE "</html>\n";

# Clean up
close (LOGFILE);
close (OUTFILE);
exit 0;

sub output_scene
{
    # Output html table row
    print OUTFILE "    <tr>\n";
    print OUTFILE "        <td>${scene}</td>\n";
    print OUTFILE "        <td>${primitives}</td>\n";
    print OUTFILE "        <td>${lights}</td>\n";
    print OUTFILE "        <td>${parser_time}</td>\n";
    print OUTFILE "        <td>${render_time}</td>\n";
    print OUTFILE "    </tr>\n";
}
