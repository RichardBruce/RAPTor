#!/usr/bin/perl

# Open input file
($#ARGV != 1)  or die "One arguement expect " . $#ARGV . " given\n";
open (LOGFILE, "<", $ARGV[0]) or die "Can't find input file";

# Output html header
print "<!DOCTYPE html>\n";
print "<html>\n";

print "<head>\n";
print "<style>\n";
print "table, th, td {\n";
print "    border: 1px solid black;\n";
print "    border-collapse: collapse;\n";
print "}\n";
print "th, td {\n";
print "    padding: 5px;\n";
print "}\n";
print "</style>\n";
print "</head>\n";

print "<body>\n";
print "<table style=\"width:100%\">\n";
print "    <tr>\n";
print "        <th>Scene</th>\n";
print "        <th># Primitives</th>\n";
print "        <th># Lights</th>\n";
print "        <th>Parser Time ms</th>\n";
print "        <th>Render Time ms</th>\n";
print "    </tr>\n";

my $scene       = "No Scenes Found";
my $primitives  = "--";
my $lights      = "--";
my $parser_time = "--";
my $render_time = "--";
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
print "</table>\n";

print "</body>\n";
print "</html>\n";

# Clean up
close (LOGFILE);
exit 0;

sub output_scene
{
    # Output html table row
    print "    <tr>\n";
    print "        <td>${scene}</td>\n";
    print "        <td>${primitives}</td>\n";
    print "        <td>${lights}</td>\n";
    print "        <td>${parser_time}</td>\n";
    print "        <td>${render_time}</td>\n";
    print "    </tr>\n";
}