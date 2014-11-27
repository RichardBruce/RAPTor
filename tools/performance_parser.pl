#!/usr/bin/perl
use strict;
use Getopt::Long;
use Cwd;

use DateTime::Format::Strptime;

my $parser = DateTime::Format::Strptime->new(
    pattern => '%Y-%m-%d',
    on_error => 'croak',
);

# Get inputs
my $log_file;
my $test_dir;
my $website_dir;
GetOptions( "website_dir=s" => \$website_dir,
            "test_dirs=s"   => \$test_dir,
            "log_file=s"    => \$log_file) or die("Error: Cant parse command line arguments\n");

###############################################
# Check there is atleast 1 output file
###############################################
my @log_files;
my @test_dirs = split(',', $test_dir);

for (@test_dirs)
{
    # Check the file exists
    my $log_path = "$_/$log_file";
    if (-e $log_path)
    {
        # Check the file isnt empty
        open (CHECK_DATA, "<", $log_path) or die "Error: Can't find project index.html ($log_path)";
        my $line = <CHECK_DATA>;
        if ($line ne "")
        {
            push @log_files, $log_path;
        }
    }
}

if ($#log_files < 0)
{
    print "No log files found\n";
    exit 0;
}

###############################################
# Create output dir if needed
###############################################
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
my $date        =  ($year + 1900) . "-" . ($mon + 1) . "-" . $mday;
my $today_date  = $parser->parse_datetime($date);
my $output_dir  = $website_dir . $project_name . "/" .  $date;
`mkdir -p $output_dir`;

###############################################
# Amend the index with a link to todays data
###############################################
my $project_index = $website_dir . $project_name . "/index.html";
my $new_project_index = $website_dir . $project_name . "/index1.html";
open (INDEX_T, "<", $project_index) or die "Error: Can't find project index.html ($project_index)";
open (INDEX_F, ">", $new_project_index) or die "Error: Can't open new project index.html";
while (my $line = <INDEX_T>)
{
    chomp;

    # Search for the historic section
    if ($line =~ m/<h2>Historic<\/h2>/)
    {
        print INDEX_F $line;

        # Skip table headers
        $line = <INDEX_T>;      # <table style="width:100%">
        print INDEX_F $line;
        $line = <INDEX_T>;      # <tr>
        print INDEX_F $line;
        $line = <INDEX_T>;      # <th>Date</th><th>Hash</th><th>Comments</th>
        print INDEX_F $line;
        $line = <INDEX_T>;      # </tr>
        print INDEX_F $line;
        $line = <INDEX_T>;      # <tr>
        print INDEX_F $line;

        # Check if todays date was already added, if so skip it
        $line = <INDEX_T>;
        if ($line =~ m/$date/)
        {
            $line = <INDEX_T>;      # </tr>
            $line = <INDEX_T>;      # <tr>
            $line = <INDEX_T>;
        }

        # Parse date
        my $days_since_run;
        if ($line =~ m/(\d{4}-\d{2}-\d{2})/)
        {
            my $last_date = $parser->parse_datetime($1);
            my $dur = $today_date->delta_days($last_date);
            $days_since_run = $dur->delta_days;
        }
        else
        {
            die "Error: Cant find date in index: $line\n";
        }

        # Get git log
        my @git_log = `git log --since=${days_since_run}.day --pretty=oneline`;

        # Pull out the latest hash
        my $git_hash;
        if ($git_log[0] =~ m/(\w+)\s/)
        {
            $git_hash = $1;
        }
        else
        {
            die "Error: Cant find hash in git log\n";
        }

        # Filter the performance related messages
        my @perf_messages;
        foreach (@git_log)
        {
            if (m/PERF:\s+([\w\s\.]+)$/)
            {
                push @perf_messages, $1;
            }
        }

        print INDEX_F "            <td><a href=\"./$date\">$date</a></td><td>$git_hash</td><td>" . join(', ', @perf_messages) , "</td>\n";
        print INDEX_F "        </tr>\n";
        print INDEX_F "        <tr>\n";
    }

    print INDEX_F $line;
}
close (INDEX_T);
close (INDEX_F);

`mv  $new_project_index $project_index`;

###############################################
# Write new output file
###############################################
open (OUTFILE, ">", $output_dir . "/index.html") or die "Error: Can't open output file: $output_dir";

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
print OUTFILE "<h1>Performance as of $date</h1>\n";

# Open input file and parse
my %headers;
my ( $scene, $primitives, $lights, $parser_time, $render_time );
for (@log_files)
{
    open (LOGFILE, "<", $_) or die "Error: Can't find input file";

    $_ =~ m/(\w+)/;

    open (CSVFILE, ">", $output_dir . "/$1.csv") or die "Error: Can't open output file: $output_dir/$1.csv";
    print OUTFILE "<h2>$1</h2>\n";
    print OUTFILE "<table style=\"width:100%\">\n";
    print OUTFILE "    <tr>\n";
    print OUTFILE "        <th>Scene</th>\n";
    print OUTFILE "        <th># Primitives</th>\n";
    print OUTFILE "        <th># Lights</th>\n";
    print OUTFILE "        <th>Parser Time ms</th>\n";
    print OUTFILE "        <th>Render Time ms</th>\n";
    print OUTFILE "    </tr>\n";
    
    print CSVFILE "Scene,# Primitives,# Lights,Parser Time ms,Render Time ms\n";

    $scene       = "No Scenes Found";
    $primitives  = "--";
    $lights      = "--";
    $parser_time = "--";
    $render_time = "--";
    while (<LOGFILE>)
    {
        chomp;
        if (m/PERF - Scene:\s+.*\/(\w+\/\w+\/\w+\.\w+)/)
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
        elsif (m/PERF - # Primitives:\s+(\d+)/)
        {
            # Get primitive counts
            $primitives = $1;
        }
        elsif (m/PERF - # Lights:\s+(\d+)/)
        {
            # Get light counts
            $lights     = $1;
        }
        elsif (m/PERF - Parser Time ms:\s+(\d+)/)
        {
            # Get parser run time
            $parser_time = $1;
        }
        elsif (m/PERF - Render Time ms:\s+(\d+)/)
        {
            # Get rendering run time
            $render_time = $1;
        }
    }

    # Output the last scene
    output_scene();

    print OUTFILE "</table>\n";
}

# Output html footer
print OUTFILE "</body>\n";
print OUTFILE "</html>\n";

# Clean up
close (CSVFILE);
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

    print CSVFILE "${scene},${primitives},${lights},${parser_time},${render_time}\n";
}
