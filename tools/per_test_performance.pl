#!/usr/bin/perl
use strict;
use Getopt::Long;
use Cwd;

# Check usage
my $test_dir;
my $website_dir;
my $number_of_commits;
GetOptions( "website_dir=s" => \$website_dir,
            "test_dirs=s"   => \$test_dir,
            "commits=i"     => \$number_of_commits) or die("Error: Cant parse command line arguments\n");

###############################################
# Get project name
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

# Get todays date
my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) = localtime();
my $date        =  ($year + 1900) . "-" . ($mon + 1) . "-" . $mday;

###############################################
# Get all historic directories, in date order
###############################################
my $output_dir  = $website_dir . $project_name;
opendir(DIR, $output_dir) or die $!;

my @files;
while (my $sub_dir = readdir(DIR))
{
    if ($sub_dir =~ m/\d{4}-\d{2}-\d{2}/)
    {
        push @files, $sub_dir;
    }
}
my @ordered_files = sort @files;
closedir(DIR);

###############################################
# Parse the newest date looking for tests
###############################################
my $newest_data = pop @ordered_files;
my @test_dirs = split(',', $test_dir);
for (@test_dirs)
{
    next if (!(-e "$output_dir/$newest_data/$_.csv"));

    # Get headers
    my $test_file = $_;
    open (NEWEST, "<", "$output_dir/$newest_data/$test_file.csv") or die "Error: Can't open newest data ($output_dir/$newest_data/$test_file.csv)\n";
    my $line = <NEWEST>;
    chomp $line;

    my @ordered_headers;
    my @split_headers = split(',', $line);
    for (@split_headers)
    {
        if (m/[umn]s$/)
        {
            push @ordered_headers, $_;
        }
    }

    my %headers = ();
    for (my $i = 0; $i <= $#ordered_headers; $i++)
    {
        $headers{$ordered_headers[$i]} = $i;
    }

    # Parse data
    while (my $line = <NEWEST>)
    {
        # Get the line into a hash
        chomp $line;
        my %datas = ();
        my @ordered_datas = split(',', $line);
        for (my $i = 0; $i <= $#ordered_datas; $i++)
        {
            $datas{$ordered_headers[$i]} = $ordered_datas[$i];
        }

        # Open output files
        my $test = $datas{"Test"};
        my $test_output_dir = "$output_dir/$test";
        `mkdir -p $test_output_dir`;
        open my $HTML, ">", $test_output_dir . "/index.html" or die "Error: Can't open output data (" . $test_output_dir . "/index.html)";
        open my $CSV, ">", $test_output_dir . "/index.csv" or die "Error: Can't open output data (" . $test_output_dir . "/index.csv)";

        # Add header
        print $HTML "<!DOCTYPE html>\n";
        print $HTML "<html>\n";
        print $HTML "<head>\n";
        print $HTML "<style>\n";
        print $HTML "table, th, td {\n";
        print $HTML "    border: 1px solid black;\n";
        print $HTML "    border-collapse: collapse;\n";
        print $HTML "}\n";
        print $HTML "th, td {\n";
        print $HTML "    padding: 5px;\n";
        print $HTML "}\n";
        print $HTML "</style>\n";
        print $HTML "</head>\n";
        print $HTML "<body>\n";
        print $HTML "<table style=\"width:100%\">\n";

        # Headers from headers hash
        print $HTML "    <tr>\n";
        print $HTML "        <th>Date</th>\n";
        for (@ordered_headers)
        {
            print $HTML "        <th>$_</th>\n";
        }
        print $HTML "    </tr>\n";
        print $CSV "Date," . join(',', @ordered_headers) . "\n";

        # Data from this file
        output_row(\%datas, $HTML, $CSV, $newest_data, @ordered_headers);

        # Find data from other files
        for (my $i = 0; $i < $number_of_commits - 1; $i++)
        {
            last if ($i > $#ordered_files);
            parse_historic($HTML, $CSV, $ordered_files[-$i - 1], $test, $test_file, @ordered_headers);
        }

        # Add footer
        print $HTML "</table>\n";
        print $HTML "</body>\n";
        print $HTML "</html>\n";

        # Done
        close($HTML);
        close($CSV);
    }
}

exit 0;

sub parse_historic
{
    my $HTML = shift;
    my $CSV = shift;
    my $date = shift;
    my $test = shift;
    my $file = shift;
    my @ordered_headers = @_;

    # Search through this historic file
    my $in_file = "$output_dir/$date/$file.csv";
    open (IN_FILE, "<", $in_file) or die "Error: Can't open newest data ($in_file)";

    # Grab the headers for this particular file
    my $line = <IN_FILE>;
    chomp $line;
    
    my %headers = ();
    my @local_ordered_headers = split(',', $line);
    for (my $i = 0; $i <= $#local_ordered_headers; $i++)
    {
        $headers{$local_ordered_headers[$i]} = $i;
    }

    # Work through the file to parse our current test
    while ($line = <IN_FILE>)
    {
        chomp $line;

        # Find the test we are working on
        my @ordered_datas = split(',', $line);
        if ($ordered_datas[$headers{"Test"}] eq $test)
        {

            my %datas = ();
            my @ordered_datas = split(',', $line);
            for (my $i = 0; $i <= $#ordered_datas; $i++)
            {
                $datas{$ordered_headers[$i]} = $ordered_datas[$i];
            }

            # Table data from this file
            output_row(\%datas, $HTML, $CSV, $date, @ordered_headers);
            last;
        }
    }

    close (IN_FILE);
}

sub output_row
{
    my $datas_ref = shift;
    my $HTML = shift;
    my $CSV = shift;
    my $date = shift;

    print $HTML "    <tr>\n";
    print $HTML "        <td>$date</td>\n";
    print $CSV "$date";
    for (@_)
    {
        # Output html table row
        print $HTML "        <td>$datas_ref->{$_}</td>\n";
     
        # Output csv
        print $CSV ",$datas_ref->{$_}"
    }
    print $HTML "    </tr>\n";
    print $CSV "\n";
}
