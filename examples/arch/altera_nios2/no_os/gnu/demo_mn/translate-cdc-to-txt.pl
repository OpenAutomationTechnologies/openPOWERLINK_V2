#!/usr/bin/perl
$bin_file=$ARGV[0];
$txt_file=$ARGV[1];

open( BINARYDATA, '<', $bin_file) or die "Unable to open file $bin_file";
open( TXTDATA, '>:raw', $txt_file) or die "Unable to open file $txt_file";

my $i=0;

while (<BINARYDATA>)
{
    $new_mem_line = $_;

    @split_mem_lines = ( $new_mem_line =~ /.{1,1}/gs );

    foreach $line(@split_mem_lines)
    {
        $num = ord $line;
        printf TXTDATA "0x%02X", $num;
        print TXTDATA ",";
        $i++;
        if(($i%16) == 0)
        {
            printf TXTDATA "\n";
        }
        else
        {
            print TXTDATA " ";
        }
    }
}

close(BINARYDATA) || die "Cannot close file!";
close(TXTDATA) || die "Cannot close file!";

print "Done writing txt file...\n";

exit;