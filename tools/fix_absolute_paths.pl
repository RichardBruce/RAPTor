#!/usr/bin/env perl

use strict;
use Getopt::Long;

my ($help, $in_file, $out_file, $format, $line, @bytes_in, @bytes_out);
                
GetOptions (
    "help"          => \$help,
    "in:s"          => \$in_file,
    "out:s"         => \$out_file,
    "format:s"      => \$format
    );

# Check arguements
if ($help or !$in_file or !$out_file or !$format)
{
    print "Usage fix_absoloute_paths -in <file> -out <file> -format <format>\n";
    print "   -in       : Specify the input file name\n";
    print "   -out      : Specify the output file name\n";
    print "   -format   : Specify the file format, current supported file formats are lwo\n";
    print "\n";
    exit;
}

# Open files
open(IN,  "< $in_file")     or die "Error: Cannot open input file $in_file";
open(OUT, "> $out_file")    or die "Error: Cannot open output file $out_file";

# Parse based on format
if ($format =~ m/lwo/)
{
    my $out_index       = 0;
    my $surf_size_ptr   = 0;
    my $surf_size       = 0;

    # Read input
    while ($line = <IN>)
    {
        push @bytes_in, split(//, $line);
    }

    for (my $i = 0; $i <= $#bytes_in; $i++)
    {
        # Bytewise copy
        $bytes_out[$out_index++] = $bytes_in[$i];

        # Check for SURF or CLIP
        # If found record length and position of SURF chunk
        if ((($bytes_in[$i] . $bytes_in[$i + 1] . $bytes_in[$i + 2] . $bytes_in[$i + 3]) =~ m/SURF/) || (($bytes_in[$i] . $bytes_in[$i + 1] . $bytes_in[$i + 2] . $bytes_in[$i + 3]) =~ m/CLIP/))
        {
            $surf_size_ptr = $out_index + 3;
            $surf_size  = 0;
            $surf_size += (ord($bytes_in[$i + 4]) << 24);
            $surf_size += (ord($bytes_in[$i + 5]) << 16);
            $surf_size += (ord($bytes_in[$i + 6]) <<  8);
            $surf_size +=  ord($bytes_in[$i + 7]);
        }
        
        # Check for TIMG or STIL
        if ((($bytes_in[$i] . $bytes_in[$i + 1] . $bytes_in[$i + 2] . $bytes_in[$i + 3]) =~ m/TIMG/) || (($bytes_in[$i] . $bytes_in[$i + 1] . $bytes_in[$i + 2] . $bytes_in[$i + 3]) =~ m/STIL/))
        {
            # Output the start code
            my $end_cnt = $i + 4;
            $i++;
            for ($i; $i < $end_cnt + 1; $i++)
            {
                $bytes_out[$out_index++] = $bytes_in[$i];
            }
            $i++;
            
            # Read in the word to work on
            my $word_of_interest = "";
            while ($bytes_in[$i] !~ /\0/)
            {
                $word_of_interest = $word_of_interest . $bytes_in[$i++];
            }
            
            # Skip the trailing nulls
            if ($bytes_in[$i + 1] =~ /\0/)
            {
                $i++;
            }
            
            # Adjust the string to a relative path
            print "TIMG: $word_of_interest\n";
            $surf_size -= length($word_of_interest);
            $word_of_interest =~ s/.*[\/]//;
            $word_of_interest =~ s/\s/_/g;
            $word_of_interest =~ s/�/e/g;
            if ($word_of_interest ne "(none)")
            {
                $word_of_interest = "./" . $word_of_interest;
            }
            
            # Null terminate the path and make it an even number of bytes long
            $word_of_interest = $word_of_interest . chr(0);
            if ((length($word_of_interest) % 2) != 0)
            {
                $word_of_interest = $word_of_interest . chr(0);
            }
            my $length_of_path = length($word_of_interest);

            # Change the length of the TIMG chunk
            $bytes_out[$out_index  ] = chr( $length_of_path       & 0xff);
            $bytes_out[$out_index-1] = chr(($length_of_path >> 8) & 0xff);
            
            # Change the SURF length to reflect our shorter file name
            $surf_size += $length_of_path - 2;
            $bytes_out[$surf_size_ptr    ] = chr(($surf_size >> 24) & 0xff);
            $bytes_out[$surf_size_ptr + 1] = chr(($surf_size >> 16) & 0xff);
            $bytes_out[$surf_size_ptr + 2] = chr(($surf_size >>  8) & 0xff);
            $bytes_out[$surf_size_ptr + 3] = chr( $surf_size        & 0xff);
    
            # Append the new file name
            print "New TIMG: $word_of_interest\n";
            push @bytes_out, split(//, $word_of_interest);
            $out_index += $length_of_path + 1;
        }
    }
    
    # Set bytes 4-7 with the new file length
    my $number_of_bytes_out = $out_index - 8;
    $bytes_out[4] = chr(($number_of_bytes_out >> 24) & 0xff);
    $bytes_out[5] = chr(($number_of_bytes_out >> 16) & 0xff);
    $bytes_out[6] = chr(($number_of_bytes_out >>  8) & 0xff);
    $bytes_out[7] = chr( $number_of_bytes_out        & 0xff);

    # Print output
    print OUT @bytes_out;
}
else
{
    print "Error: Unknown file format\n";
    exit 1;
}

# Close files
close IN;
close OUT;

exit 0;
