#ver 1.03
#added .mem file capability.
#added MEM swap option
#

####*****************************************************************************************
####**
####**  Disclaimer: LIMITED WARRANTY AND DISCLAMER. These designs are
####**              provided to you "as is". Xilinx and its licensors make and you
####**              receive no warranties or conditions, express, implied, statutory
####**              or otherwise, and Xilinx specifically disclaims any implied
####**              warranties of merchantability, non-infringement, or fitness for a
####**              particular purpose. Xilinx does not warrant that the functions
####**              contained in these designs will meet your requirements, or that the
####**              operation of these designs will be uninterrupted or error free, or
####**              that defects in the Designs will be corrected. Furthermore, Xilinx
####**              does not warrant or make any representations regarding use or the
####**              results of the use of the designs in terms of correctness, accuracy,
####**              reliability, or otherwise.
####**
####**              LIMITATION OF LIABILITY. In no event will Xilinx or its licensors be
####**              liable for any loss of data, lost profits, cost or procurement of
####**              substitute goods or services, or for any special, incidental,
####**              consequential, or indirect damages arising from the use or operation
####**              of the designs or accompanying documentation, however caused and on
####**              any theory of liability. This limitation will apply even if Xilinx
####**              has been advised of the possibility of such damage. This limitation
####**              shall apply not-withstanding the failure of the essential purpose of
####**              any limited remedies herein.
####**
####*****************************************************************************************

use Getopt::Long;	# for getopt
use File::Copy;		# for file move and copy



print "**********************************\n";
print "*\n";
print "*	Preliminary script v1.03\n";
print "*	Author: Shalin Sheth\n";
print "*	Adapted: Thomas Mair\n";
print "*\n";
print "**********************************\n\n";



# Software section synchronization word
$sync_word = "9F8FAFBF";

# user data section syncronization word
$sync_word_2 = "8F9FAFBF";

################################
################################

#Define USAGE of script
sub usage
{
	print "usage:\n
		pc.pl [--format : outfile format {mcs|hex|bin}] [--swap : bit swap {on|off}] [--memfile : user data file {<filename.ext>}] [--promfile : PROM file {filename.{mcs|hex}}] [--outfile : PROM out file {filename.ext}]\n
		--help   = print this help page\n

		--format = PROM file format used
			mcs  => Intel file format
			hex  => Simple hex file format
			bin  => Format .mem file and write to binary file\n

		--swap  = Specify if bits are to be swapped
			on  => swaps bits in every byte
			off => bits are not swapped\n

		--memfile  = File containing user data to be added to PROM file\n

		--promfile = PROM file to which the user data should be added\n

		--outfile  = PROM file to which the output data to be written\n\n";
}

#
#
#
#
################################
################################

# check arguments
#-- prints usage if no command line parameters are passed or there is an unknown
#   parameter or help option is passed
if ( @ARGV < 1 or
     !GetOptions('help|?' => \$help, 'format=s' => \$format, 'swap=s' => \$do_swap, 'memfile=s' => \$user_file, 'promfile=s' => \$prom_file, 'outfile=s' => \$out_file, )
     or defined $help )
{
	usage();
	exit;
}

if ($format eq "")
{
	print "ERROR: No --format parameter provided!\n";
	usage();
	exit;
} elsif ($user_file eq "")
{
	print "ERROR: No --memfile parameter provided!\n";
	usage();
	exit;
} elsif ($out_file eq "")
{
	print "ERROR: No --outfile parameter provided!\n";
	usage();
	exit;
}

#
#
#
#
################################
################################

#Detect whether user data is a mem file

@memfilecheck = split(/\./, $user_file);

if (@memfilecheck[1] eq "mem")
{
	print "Formatting MEM file...\n";
	open (MEMFILE, $user_file) || die "Cannot open MEM file $user_file: $!";
	open (USERDATA, ">@memfilecheck[0]\.tmp") || die "Cannot create file @memfilecheck[0].tmp: $!";

	# print sync word to userdata file
	print USERDATA $sync_word;

	$char_count = 4;

	while (<MEMFILE>)
	{
		if ($_ =~ /^\/\/ Program/)
		{
			# found size comment
			print "Found size comment...		\n";

			$size_string = $_;

			@split_size = split (/ /, $size_string);

			$size_hex = @split_size[7];

			$size = sprintf("%08X", hex($size_hex));
		}
		elsif ($_ =~ /^\/\//)
		{
			# ignore comment
			print "Ignoring comments...		\n";
		}
		elsif ($_ =~ /^@/)
		{
			# address detected
			print "Formating address...		\n";

			$address = $_;

			chomp($address);

			$address =~ s/@//g;

			# print address to file
			@groups = ( $address =~ /.{1,2}/gs );

			foreach $byte(@groups)
			{
				print USERDATA $byte;

				$char_count++;

				if ($char_count == 16)
				{
					print USERDATA "\n";
					$char_count = 0;
				}
			}

			# print size to file
			@groups = ( $size =~ /.{1,2}/gs );

			foreach $byte(@groups)
			{

				print USERDATA $byte;

				$char_count++;

				if ($char_count == 16)
				{
					print USERDATA "\n";
					$char_count = 0;
				}
			}

		}
		else
		{
			#data stream detected
			$new_mem_line = $_;

			$new_mem_line =~ s/^\s+//;
			$new_mem_line =~ s/\s+$//;

			@split_mem_lines = split (/ /, $new_mem_line);

			foreach $line(@split_mem_lines)
			{
				print USERDATA $line;

				$char_count++;

				if ($char_count == 16)
				{
					print USERDATA "\n";
					$char_count = 0;
				}
			}
		}
	}

	# write good by zeros
	@split_end = ( "0000000000000000" =~ /.{1,2}/gs );

	foreach $byte(@split_end)
	{

		print USERDATA $byte;

		$char_count++;

		if ($char_count == 16)
		{
			print USERDATA "\n";
			$char_count = 0;
		}
	}

	# fill last line with zeros to 16
	$char_rest = 16 - $char_count;
	for ($i = 0; $i < $char_rest; $i++)
	{
		print USERDATA "00";
	}
	print USERDATA "\n";

	close (MEMFILE) || die "Cannot close file $user_file: $!";
	close (USERDATA) || die "Cannot close file @memfilecheck[0]\.tmp: $!";

} else {
	print "ERROR: Wrong user file format!\n";
	die;
}
	$user_file = "@memfilecheck[0].tmp";


if ($format eq "bin")
{
	# just write application software to a binary file
	# reopen user file
	open (USERDATA, "<@memfilecheck[0]\.tmp") || die "Cannot create file @memfilecheck[0].tmp: $!";
	# open binary file
	open( BINARYDATA, '>:raw', $out_file) or die "Unable to open: <$out_file>";

	while (<USERDATA>)
	{
		$new_mem_line = $_;

		$new_mem_line =~ s/^\s+//;
		$new_mem_line =~ s/\s+$//;

		@split_mem_lines = ( $new_mem_line =~ /.{1,2}/gs );

		foreach $line(@split_mem_lines)
		{
			$dec_byte = hex $line;
			print BINARYDATA pack("c", $dec_byte) ;
		}
	}

	close(BINARYDATA) || die "Cannot close file $out_file: $!";
	close(USERDATA) || die "Cannot close file @memfilecheck[0]\.tmp: $!";

	print "Done writing binary file <$out_file>...\n";
	exit;
}


################################
################################
#Initialise all variables

$prom_line_number = 0;
$user_line_number = 0;
#
#
#
#
################################
################################

#Print settings used
print "\n\r";
print "Running script with following settings:\n";
print "	PROM file format		==>	$format\n";
print "	Bit swapping			==>	$do_swap\n";
print "	User data file			==>	$user_file\n";
print "	Original PROM file		==>	$prom_file\n";
print "	Temporary PROM file		==>	$prom_file.tmp\n";
print "	Output file is			==>	$out_file\n\n";
#
#
#
#
################################
################################

#Open files and begin processing

if (-e $prom_file and $format eq "mcs")
{
	open (PROM_FILE, "<$prom_file") || die "Cannot open file $prom_file: $!";
} else {
	print "INFO: No PROM file provided! Creating an empty file!\n";
	open (PROM_FILE, "+>$prom_file");

	# print start and end section to prom file
	print PROM_FILE ":020000040000FA\n";
	print PROM_FILE ":00000001FF\n";
}

open (PROM_FILE_TMP, '>', "$prom_file.tmp") || die "Cannot open file <$prom_file.tmp>: $!";
open (USER_DATA, '<', $user_file) || die "Cannot open file <$user_file>: $!";

print "Copying original PROM and USERDATA to <$prom_file.tmp> ...\n";

while (<PROM_FILE>)
{
	print("\nWARNING: User Sync Word $sync_word not found\n") if /$sync_word/;
	print("\nWARNING: User Sync Word $sync_word_2 not found\n") if /$sync_word_2/;

	$current_prom_line = $_;

	#Process "mcs" file format

	if ($format eq "mcs")
	{
		#print "Copying original PROM line number $prom_line_number...		\r";

		$prom_line_number = $prom_line_number + 1;

		#Get the last information for record type "04"
		get_current_mcs_prom_line_data();


		#If the current PROM line is not the last line in the PROM
    	#file then just print it into the new PROM file. If the line
    	#is the last line in the PROM file then we take the previous
    	#line and find its address, so we can add a new line, with
    	#new data at the next address. Also, we calculate a checksum
    	#for the new line in the PROM file and append that to the new
    	#PROM line.

		if ($current_prom_line =~ /\:00000001FF$/)
		{
			print "\n";
			#New address offset starts at 0
			$new_address_offset = 0;
			#Add user data to PROM file
			while (<USER_DATA>)
			{
				chomp;

				if ($_ =~ /^\#/)
				{
					#Ignoring comments
					print "Ignoring comment\r";
				}
				elsif ($_ =~ /\#/g)
				{
					@split_user_line = split(/\#/, $_);
					$current_user_line = @split_user_line[0];
					#print "Processing USER line $user_line_number...		\r";

					print $current_user_line;
					print "\n";

					##new
					#Extract bytes from user data

    				(@bytes_hex) = unpack("A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2", $current_user_line);

    				foreach $byte_hex(@bytes_hex)
    				{

		    			#Convert hex to decimal
    					$byte_dec = hex($byte_hex);

    					#Convert decimal to binary
    					$byte_binary = decimal2binary($byte_dec);

    					#Get the last eight bits
    					$last_eight_bits = substr($byte_binary, -8);

    					#Extract each bit
    					(@last_eight_bits_not_swapped) = unpack("A1 A1 A1 A1 A1 A1 A1 A1", $last_eight_bits);

    					#Bit swap each bit
    					@last_eight_bits_swapped = reverse(@last_eight_bits_not_swapped);
    					$byte_swapped_bin = 0;

    					#Concatenate the bits to form a byte

    					foreach $bit(@last_eight_bits_swapped)
    					{
		    				$byte_swapped_bin = $byte_swapped_bin.$bit;
    					}

    					#Convert binary to decimal
    					$byte_swapped_dec = binary2decimal($byte_swapped_bin);

    					#Convert decimal to hex
    					$byte_swapped_hex = sprintf "%lx", $byte_swapped_dec;

    					#Get the last byte
    					$byte_hex = substr($byte_swapped_hex, -2);

    					#If the value is less than 0x0F then concatenate a "0" to the front
    					if ($byte_swapped_dec <= 15)
    					{
		    				$byte_hex = "0$byte_hex";
    					}


						$new_current_user_line = $new_current_user_line.$byte_hex;
    				}

					if($do_swap eq "on")
					{
						$current_user_line=$new_current_user_line;
					}

					$new_current_user_line = "";##new

					$user_line_number = $user_line_number + 1;

					#Calculate the new address

					get_mcs_address();

				}
				else
				{
					$current_user_line = $_;

					#print "Processing USER line $user_line_number...		\r";

					#Extract bytes from user data

    				(@bytes_hex) = unpack("A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2", $current_user_line);

    				foreach $byte_hex(@bytes_hex)
    				{
		    			#Convert hex to decimal
    					$byte_dec = hex($byte_hex);

    					#Convert decimal to binary
    					$byte_binary = decimal2binary($byte_dec);

    					#Get the last eight bits
    					$last_eight_bits = substr($byte_binary, -8);

    					#Extract each bit
    					(@last_eight_bits_not_swapped) = unpack("A1 A1 A1 A1 A1 A1 A1 A1", $last_eight_bits);

    					#Bit swap each bit
    					@last_eight_bits_swapped = reverse(@last_eight_bits_not_swapped);
    					$byte_swapped_bin = 0;

    					#Concatenate the bits to form a byte
    					foreach $bit(@last_eight_bits_swapped)
    					{
		    				$byte_swapped_bin = $byte_swapped_bin.$bit;
    					}

    					#Convert binary to decimal
    					$byte_swapped_dec = binary2decimal($byte_swapped_bin);

    					#Convert decimal to hex
    					$byte_swapped_hex = sprintf "%lx", $byte_swapped_dec;

    					#Get the last byte
    					$byte_hex = substr($byte_swapped_hex, -2);

    					#If the value is less than 0x0F then concatenate a "0" to the front
    					if ($byte_swapped_dec <= 15)
    					{
		    				$byte_hex = "0$byte_hex";
    					}

##    					print PROM_FILE_TMP uc("$byte_hex");

						$new_current_user_line = $new_current_user_line.$byte_hex;

    				}

					if($do_swap eq "on")
					{
						$current_user_line=$new_current_user_line;
					}

					$new_current_user_line = "";
					$user_line_number = $user_line_number + 1;

					#Calculate the new address
					get_mcs_address();
				}
			}

			print PROM_FILE_TMP $current_prom_line;
		}
		else
		{
			#Print the line to the new PROM file unchanged
			print PROM_FILE_TMP $current_prom_line;

			#Store the current line for use in next iteration
			$previous_prom_line = $current_prom_line;
		}
	}

	elsif ($format eq "hex")
	{
	    #Print the original PROM file contents to the new PROM file.
	    if ($do_swap eq "on")
	    {
    		$current_prom_line = $_;

	    	print "Copying original PROM line number $prom_line_number...\r";

			$prom_line_number = $prom_line_number + 1;

	    	print PROM_FILE_TMP $current_prom_line;

	    	print "\n";

    		while (<USER_DATA>)
    		{
	    		if ($_ =~ /^\#/)
	    		{
		    		print "Ignoring comment\r";
	    		}
	    		elsif ($_ =~ /\#/g)
	    		{
		    		chomp;

		    		@split_user_line = split(/\#/, $_);

		    		$current_user_line = @split_user_line[0];

	    			print "Processing USER line $user_line_number...			\r";

					$user_line_number = $user_line_number + 1;

		   			#Extract bytes from user data
    				(@bytes_hex) = unpack("A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2", $current_user_line);

    				foreach $byte_hex(@bytes_hex)
    				{
		    			#Convert hex to decimal
    					$byte_dec = hex($byte_hex);

    					#Convert decimal to binary
    					$byte_binary = decimal2binary($byte_dec);

    					#Get the last eight bits
    					$last_eight_bits = substr($byte_binary, -8);

    					#Extract each bit
    					(@last_eight_bits_not_swapped) = unpack("A1 A1 A1 A1 A1 A1 A1 A1", $last_eight_bits);

    					#Bit swap each bit
    					@last_eight_bits_swapped = reverse(@last_eight_bits_not_swapped);
    					$byte_swapped_bin = 0;

    					#Concatenate the bits to form a byte
    					foreach $bit(@last_eight_bits_swapped)
    					{
		    				$byte_swapped_bin = $byte_swapped_bin.$bit;
    					}

    					#Convert binary to decimal
    					$byte_swapped_dec = binary2decimal($byte_swapped_bin);

    					#Convert decimal to hex
    					$byte_swapped_hex = sprintf "%lx", $byte_swapped_dec;

    					#Get the last byte
    					$byte_hex = substr($byte_swapped_hex, -2);

    					#If the value is less than 0x0F then concatenate a "0" to the front
    					if ($byte_swapped_dec <= 15)
    					{
		    				$byte_hex = "0$byte_hex";
    					}

    					print PROM_FILE_TMP uc("$byte_hex");
    				}
				}
				else
				{
					chomp;

					$current_user_line = $_;
					print "Processing USER line $user_line_number...			\r";

					$user_line_number = $user_line_number + 1;

		   			#Extract bytes from user data
    				(@bytes_hex) = unpack("A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2 A2", $current_user_line);

    				foreach $byte_hex(@bytes_hex)
    				{

		    			#Convert hex to decimal
    					$byte_dec = hex($byte_hex);

    					#Convert decimal to binary
    					$byte_binary = decimal2binary($byte_dec);

    					#Get the last eight bits
    					$last_eight_bits = substr($byte_binary, -8);

    					#Extract each bit
    					(@last_eight_bits_not_swapped) = unpack("A1 A1 A1 A1 A1 A1 A1 A1", $last_eight_bits);

    					#Bit swap each bit
    					@last_eight_bits_swapped = reverse(@last_eight_bits_not_swapped);
    					$byte_swapped_bin = 0;

    					#Concatenate the bits to form a byte
    					foreach $bit(@last_eight_bits_swapped)
    					{
		    				$byte_swapped_bin = $byte_swapped_bin.$bit;
    					}

    					#Convert binary to decimal
    					$byte_swapped_dec = binary2decimal($byte_swapped_bin);

    					#Convert decimal to hex
    					$byte_swapped_hex = sprintf "%lx", $byte_swapped_dec;

    					#Get the last byte
    					$byte_hex = substr($byte_swapped_hex, -2);

    					#If the value is less than 0x0F then concatenate a "0" to the front
    					if ($byte_swapped_dec <= 15)
    					{
		    				$byte_hex = "0$byte_hex";
    					}

    					print PROM_FILE_TMP uc("$byte_hex");
    				}
				}
    		}
    	}

    	#Simply print the original file and then the user data
    	#into the new PROM file.

    	elsif ($do_swap eq "off")
    	{

    		$current_prom_line = $_;

	    	print "Copying original PROM line number $prom_line_number...\r";

			$prom_line_number = $prom_line_number + 1;

	    	print PROM_FILE_TMP $current_prom_line;

	    	print "\n";

    		while (<USER_DATA>)
    		{
	    		if ($_ =~ /^\#/)
	    		{
		    		print "Ignoring comment\r";
	    		}

	    		elsif ($_ =~ /\#/g)
	    		{
		    		chomp;
		    		@split_user_line = split(/\#/, $_);
		    		$current_user_line = @split_user_line[0];
	    			print "Processing USER line $user_line_number...			\r";
					$user_line_number = $user_line_number + 1;
    				print PROM_FILE_TMP uc("$current_user_line");
				}
				else
				{
					chomp;
					print "Processing USER line $user_line_number...			\r";
					$user_line_number = $user_line_number + 1;
    				$current_user_line = $_;
    				print PROM_FILE_TMP uc("$current_user_line");
				}
    		}
    	}
	}
}


close (USER_DATA) || die "Cannot close file $user_file: $!";
close (PROM_FILE) || die "Cannot close file $prom_file: $!";
close (PROM_FILE_TMP) || die "Cannot close file $prom_file.tmp: $!";

if ($format eq "mcs" and $prom_file eq $out_file)
{
	print "PROM file and output file are the same! Overwriting old PROM file.... ";
	if (move("$prom_file.tmp", $out_file) != 1)
	{
		print "\nError while moving <$prom_file.tmp> over <$out_file>! Reason: <$!>!\n\n";
		die;
	}

	if (unlink("$prom_file.tmp") != 0)
	{
		print "\nError while delecting <$prom_file.tmp>!\n\n";
		die;
	}
	print "DONE!\n";

}

print "DONE generating PROM file $out_file ...\n";

#
#
#
#
################################
################################

#Get the data from the current prom line
sub get_current_mcs_prom_line_data
{
	($start_character, $byte_count_hex) = unpack("A1 A2", $current_prom_line);

	#Convert byte count from hex to decimal
	$byte_count_dec = hex($byte_count_hex);
	$byte_count_dec = $byte_count_dec * 2;

	#Based on byte count get other fields from the PROM line
	(	$start_character,
		$byte_count_hex,
		$address_hex[0],
		$address_hex[1],
		$record_type_hex,
		$all_data_hex,
		$checksum_hex
	) = unpack("A1 A2 A2 A2 A2 A$byte_count_dec A2", $current_prom_line);

	#If this is a "04" record type then store its information
	if ($record_type_hex eq "04")
	{
		$last_04_record_data_hex = $all_data_hex;
	}
}

#
#
#
#
################################
################################

#Calculate the new address to be used
sub get_mcs_address
{
	#Get the address from the PROM file
	($address_hex) = unpack("x3 A4", $previous_prom_line);

	#Convert the hex address to decimal
	$address_dec = hex($address_hex);

	#Calculate new address value based on existing address value
	if ($address_dec eq "65520")
	{
		#Store current user data temporarily
		$temporary_current_user_line = $current_user_line;

		#New address starts at zero
		$new_address_dec = 0;
		$new_address_hex = "0000";
		($address_hex[0], $address_hex[1]) = unpack("A2 A2", $new_address_hex);
	    $address_dec[0] = hex($address_hex[0]);
    	$address_dec[1] = hex($address_hex[1]);
		$new_address_offset = 0;

		#Convert hex record data to decimal
		$new_04_record_data_dec = hex($last_04_record_data_hex);

		#Calculate new record "04" data
		$new_04_record_data_dec = $new_04_record_data_dec + 1;

		#Convert to hex
		$new_04_record_data_hex = sprintf "%lx", $new_04_record_data_dec;

		#Store for next use
		$last_04_record_data_hex = $new_04_record_data_hex;

		#Make data at least 4 characters long (i.e. 2 data bytes)
		$length = length($new_04_record_data_hex);

		if ($length > 4)
		{
			die "Record data is too large....Quitting: $!";
		}
		else
		{
			for ($i = 0; $i < 4 - $length; $i++)
			{
				$new_04_record_data_hex = "0$new_04_record_data_hex";
			}
		}

		#Define new record data
		$byte_count_hex = "02";
		$byte_count_dec = hex($byte_count_hex);
		$record_type_hex = "04";
		$record_type_dec = hex($record_type_hex);
		$current_user_line = $new_04_record_data_hex;

		#Calculate checksum for new "04" record
		calculate_mcs_checksum();

		#Print new "04" record to new PROM file
		print PROM_FILE_TMP uc(":$byte_count_hex$new_address_hex$record_type_hex$new_04_record_data_hex$checksum_hex\n");
		$previous_prom_line = uc(":$byte_count_hex$new_address_hex$record_type_hex$new_04_record_data_hex$checksum_hex\n");

		#Restore current user data from temporary storage
		$current_user_line = $temporary_current_user_line;
	}
	else
	{
		#Calculate the offset for the next address to be used in the PROM file
		$new_address_offset = 16;#$new_address_offset + 16;

		#Calculate the new address
		$new_address_dec = $address_dec + $new_address_offset;
	}

	#Convert new address to hex
	$new_address_hex = sprintf "%lx", $new_address_dec;

	#Make address at least 4 characters long (i.e. 2 address bytes)
	$length = length($new_address_hex);

	if ($length > 4)
	{
		die "Address is too large....Quitting: $!";
	}
	else
	{
		for ($i = 0; $i < 4 - $length; $i++)
		{
			$new_address_hex = "0$new_address_hex";
		}
	}

	($address_hex[0], $address_hex[1]) = unpack("A2 A2", $new_address_hex);
    $address_dec[0] = hex($address_hex[0]);
    $address_dec[1] = hex($address_hex[1]);
	$byte_count_dec = "16";
	$byte_count_hex = "10";
	$record_type_dec = "00";
	$record_type_hex = "00";

	#Calculate checksum for current user data
	calculate_mcs_checksum();

	#Print new data to new PROM file
	print PROM_FILE_TMP uc(":$byte_count_hex$new_address_hex$record_type_hex$current_user_line$checksum_hex\n");
	$previous_prom_line = uc(":$byte_count_hex$new_address_hex$record_type_hex$current_user_line$checksum_hex\n");
}

#
#
#
#
################################
################################

#Calculate the checksum for the new line
sub calculate_mcs_checksum
{
	$skip = 0;
	$data_sum_hex = 0;
	$data_sum_dec = 0;

	#Based on byte count get individual data bytes and their sum
	for ($d = 0; $d < $byte_count_dec; $d++)
	{
		($data_hex[$d]) = unpack("x$skip A2", $current_user_line);
		$skip = $skip + 2;

		#Convert data byte to decimal format
		$data_dec[$d] = hex($data_hex[$d]);

		#Add all data bytes together
		$data_sum_dec = $data_sum_dec + $data_dec[$d];

		#convert decimal to hex format
		$data_sum_hex = sprintf "%lx", $data_sum_dec;
	}

	#Add all fields together
	$all_sum_dec = $data_sum_dec + $byte_count_dec + $address_dec[0] + $address_dec[1] + $record_type_dec;

    #Convert the decimal sum to hex format
    $all_sum_hex = sprintf "%lx", $all_sum_dec;

    #Get the last two bytes of the hex sum
    $last_two_bytes_of_sum_hex = substr($all_sum_hex, -2);

    #Convert the last two bytes of the hex sum to decimal format
    $last_two_bytes_of_sum_dec = hex($last_two_bytes_of_sum_hex);

    #Invert the bits - 1's complement
    $inverted_dec = $last_two_bytes_of_sum_dec ^ 255;

    #Convert the 1's complement to hex format
    $inverted_hex = sprintf "%lx", $inverted_dec;

    #Get the last two bytes of 1's complement in hex format
    $last_two_bytes_of_inverted_dec = hex($inverted_hex);

    #Add 1 to last two bytes - 2's complement
    $checksum_dec = $last_two_bytes_of_inverted_dec + 1;

    #Convert 2's complement to hex format
    $last_two_bytes_2s_hex = uc(sprintf "%lx", $checksum_dec);

    #Get the last two bytes of hex 2's compliment
    ($checksum_he) = substr($last_two_bytes_2s_hex, -2);

    #If value is less than two characters then add a '0'
    if ($checksum_dec <= 15)
    {
    	$checksum_hex = "0$checksum_he";
    }
    else
    {
    	$checksum_hex = $checksum_he;
    }
}

#
#
#
#
################################
################################

#Decimal to binary representation conversion
sub decimal2binary
{
    my $bin_value = unpack("B32", pack("N", shift));
    $bin_value =~ s/^0+(?=d)//;

    return $bin_value;
}

#
#
#
#
################################
################################

#Binary to decimal representation conversion
sub binary2decimal
{
    return unpack("N", pack("B32", substr("0" x 32 . shift, -32)));
}