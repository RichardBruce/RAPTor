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
my @git_branch = `git rev-parse --abbrev-ref HEAD`;
$website_dir = $website_dir . "/" . $git_branch[0];

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
my $output_dir  = "$website_dir$project_name/$date";
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
my %headers = ();
my @header_order;
my ( $scene, $primitives, $lights, $parser_time, $render_time );
for my $input_file (@log_files)
{
    # 1 Pass of the file to find the headers and track the order they are found
    open (LOGFILE, "<", $input_file) or die "Error: Can't find input file";
    while (<LOGFILE>)
    {
        if (m/PERF (\d+) - ([\w\s#]+):/)
        {
            if ($headers{$2} != 1)
            {
                $header_order[$1] = $2;
            }
            $headers{$2} = 1;
        }
    }
    close (LOGFILE);

    # Check we have something to output
    next if ($#header_order < 0);
    
    $input_file =~ m/(\w+)/;
    open (CSVFILE, ">", "$output_dir/$1.csv") or die "Error: Can't open output file: $output_dir/$1.csv";

    # Output headers
    print OUTFILE "<h2>$1</h2>\n";
    print OUTFILE "<table style=\"width:100%\">\n";
    print OUTFILE "    <tr>\n";
    for (@header_order)
    {
        print OUTFILE "        <th>$_</th>\n";
    }
    print OUTFILE "    </tr>\n";
    
    print CSVFILE join(',', @header_order) . "\n";

    open (LOGFILE, "<", $input_file) or die "Error: Can't find input file";
    $headers{"Test"} = "No Scenes Found";
    while (my $line = <LOGFILE>)
    {
        chomp $line;

        if ($line =~ m/PERF \d+ - Test:\s+(.*)/)
        {
            # Output last scene
            if ($headers{"Test"} ne "No Scenes Found")
            {
                output_scene();
            }

            # Reset data
            for (keys %headers)
            {
                $headers{$_} = "--";
            }

            # Graph new scene name
            $headers{"Test"} = $1;
        }
        # Check if we have a value for this key
        else
        {
            for (keys %headers)
            {
                if ($line =~ m/PERF \d+ - $_:\s+(.*)/)
                {
                    $headers{$_} = $1;
                }
            }
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

###############################################
# Make graphs
###############################################
for (@test_dirs)
{
    my $plot_file = "$_/summary_plot.sh";
    if (-e $plot_file)
    {
        `$plot_file $website_dir$project_name/$date/$_.csv $website_dir$project_name/$_.png`;
    }
}

exit 0;

sub output_scene
{
    print OUTFILE "    <tr>\n";
    for (@header_order)
    {
        # Output html table row
        print OUTFILE "        <td>$headers{$_}</td>\n";
        
        # Output csv
        print CSVFILE "$headers{$_}";
        if ($_ eq $header_order[$#header_order])
        {
            print CSVFILE "\n";
        }
        else
        {
            print CSVFILE ",";
        }
    }
    print OUTFILE "    </tr>\n";
}
