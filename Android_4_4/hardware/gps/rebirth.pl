#!/usr/bin/perl
###############################################################################
#
# Copyright (C) u-blox AG
# u-blox AG, Thalwil, Switzerland
#
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any
# purpose without fee is hereby granted, provided that this entire notice
# is included in all copies of any software which is or includes a copy
# or modification of this software and in all copies of the supporting
# documentation for such software.
#
# THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
# REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
# OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
#
###############################################################################
#
# Project: Android GNSS Driver
#
###############################################################################
# $Id: rebirth.pl 114165 2016-04-27 08:31:20Z fabio.robbiani $
# $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/rebirth.pl $
###############################################################################

# This script updates the necessary files on the target.
# Use -d flag to enable debugging.

use strict;
use warnings;
use POSIX ":sys_wait_h";

use FileHandle;
use FindBin;

# enable debugging if the -d flag is specified on the command line
my $debug = ("@ARGV" =~ m/-d/ ? 1 : 0);

# find adb binary, fall back to assuming 'adb' (or 'adb.exe') is in the $PATH
# Note: using / as directory separator is fine in Windows, Perl always knows what you mean :)
my $adb = (grep { $_ && -f $_ } (
             ($ENV{'PROGRAMFILES(X86)'} ? $ENV{'PROGRAMFILES(X86)'} . '/Android/android-sdk/platform-tools/adb.exe' : undef),
             ($ENV{'PROGRAMFILES'} ? $ENV{'PROGRAMFILES'} . '/Android/android-sdk/platform-tools/adb.exe' : undef),
             '/opt/android/platform-tools/adb',
           ))[0] || 'adb';

if ($^O =~ m/win32/i)
{
    # Windows shell likes "" around path names with spaces
    $adb = '"' . $adb . '"';
}

# directory where this script lives
my $dir = $FindBin::Bin;


my @etcFiles = ('gps.conf', 'u-blox.conf', 'ca-cert-google.pem');

print("adb=$adb\ndir=$dir\n") if ($debug);

# result (exit code) and output of external commands (adb)
my $err = 0;
my @out = ();
my $product = "";


# Check if system is up and get product type
if (!$err)
{
    print("* checking if adb and the target are up\n");
    shellCmd("$adb devices", 5); # Start server if not done
    ($err, @out) = shellCmd("$adb shell \"getprop | grep ro.build.product\"", 5);
    $product = $out[0] if scalar(@out) > 0;
    if ($product =~ m/^.*error:.*$/g)
    {
        $err = 1;
        print ("ERROR: $product\n");
    }
    else
    {
        $product =~ s/\[.*\]:\s*\[(.*)\]\s*/$1/g;
    }
}

my $target = $ENV{'TARGET_PRODUCT'} ? $ENV{'TARGET_PRODUCT'} : $product;
my $lib = '../../../out/target/product/' . $target . '/system/lib/hw/gps.default.so';

if (!$err)
{
    unless (-e $lib)
    {
        print("* The file $lib does not exist. Using gps.default.so in local directory if existing\n");
        $lib = "gps.default.so"
    }
}

my $file;
my @allFiles = @etcFiles;
push @allFiles, $lib;

if (!$err)
{
    foreach $file (@allFiles)
    {
        unless (-e $file)
        {
            $err = 1;
            print("ERROR: File $file does not exist!\n");
        }
    }
}

if (!$err)
{
    foreach $file (@etcFiles)
    {
        if (!$err)
        {
            print("* install /etc/$file file\n");
            ($err, @out) = shellCmd("$adb push $dir/$file /etc/", 10);
        }
    }
}

if (!$err)
{
    if ((defined $ARGV[0]) && ($ARGV[0] eq "nodriver"))
    {
        print("* Driver not installed\n");
    }
    else
    {
        print("* install /system/lib/hw/gps.default.so library\n");
        ($err, @out) = shellCmd("$adb push $dir/$lib /system/lib/hw/", 10);
    }
}

if (!$err)
{
    if ($product eq "bamboo" || $product eq "panda")
    {
        # Special treatment for panda boards as they do not recover
        # from reboots
        my $zygotePid = -1;
        print("* find PID of the zygote task\n");
        ($err, @out) = shellCmd("$adb shell \"ps | grep zygote | grep -v grep\"", 5);

        if ($out[scalar(@out)-1] && ($out[scalar(@out)-1] =~ m/^\s*root\s+(\d+).*zygote\s*$/o))
        {
            $zygotePid = $1;
        }

        $err = 1 if ($zygotePid == -1);

        if (!$err)
        {
            print("* kill zygote (PID: $zygotePid)\n");
            ($err, @out) = shellCmd("$adb shell kill $zygotePid", 5);
        }

    }
    else # Reboot the device
    {
        print("* reboot the device\n");
        ($err, @out) = shellCmd("$adb shell reboot", 2, 1);
        $err = 0; # ADB reboot will not return.
    }
}
print "Exiting script\n" if ($debug);

# atexit() routine
END
{
    if ($err)
    {
        print("ERROR: Something went wrong above!\n");
        if ($^O =~ m/Win32/i)
        {
            print("Press any key to end.\n");
            getc();
        }
        exit($err);
    }
}

sub shellCmdCh
{
    my $cmd = shift;
    my $discardOutput = 0;
    if (scalar(@_) > 0)
    {
        $discardOutput = shift;
    }
    $cmd .= " 2>&1" unless ($cmd =~ m@\s+2>.+@g);
    my $retval;
    print("--> $cmd\n") if ($debug);
    my @cmdOut = ();
    unless (open(CMD, '-|', $cmd))
    {
        print("ERROR: Could not run command '$cmd': $!\n");
        return -1;
    }
    else
    {
        if($discardOutput == 0)
        {
            while (<CMD>)
            {
                chomp();
                push(@cmdOut, $_);
                print("<-- $_\n") if ($debug);
            }
        }
        close(CMD);
    }
    my $ret = $? || 0;
    my $coreDumped = ($ret & 0x80);
    my $signal     = ($ret & 0x7f);
    my $exitCode   = ($ret >> 8) & 0xff;

    if ($exitCode || $coreDumped || $signal)
    {
        printf("ERROR: Command '$cmd' failed (ret=0x%04x coreDumped=%i signal=0x%02x exitCode=%i)!\n",
                $ret, $coreDumped, $signal, $exitCode);
    }

    return $exitCode, @cmdOut;
}

sub shellCmd
{
    my $err;
    my @out = ();
    my $cmd = shift;
    my $discardOutput = 0;
    my $killAfterSec = 0;
    if (scalar(@_) > 0)
    {
        $killAfterSec = shift;
        if (scalar(@_) > 0)
        {
            $discardOutput = shift;
        }
    }

    pipe(READER, WRITER) || die "Creating pipe failed: $!";
    WRITER->autoflush(1);
    my $handle = fork();
    defined($handle)     || die "Fork failed: $!";

    if ($handle == 0)
    {
        # Child
        close READER;
        ($err, @out) = shellCmdCh($cmd, $discardOutput);
        my $line;
        foreach $line (@out)
        {
            print WRITER $line;
        }
        close(WRITER);
        print "CERR: $err\n" if ($debug);
        exit($err);
    }
    else
    {
        # Parent

        close WRITER;
        if ($killAfterSec == 0)
        {
            wait();
        }
        else
        {
            my $inc = 1;
            my $count;
            for($count=0; $count <= $killAfterSec; $count+=$inc)
            {
                my $status = waitpid($handle, WNOHANG);
                if($status == $handle)
                {
                    last;
                }
                else
                {
                    sleep($inc);
                }
            }
            # Has the child exited by now?
            if($count >= $killAfterSec)
            {
                print "PERR: Killing the child $handle\n" if ($debug);
                kill 9, $handle;
            }
        }
        $err = $? >> 8;
        if($err != 0)
        {
            print "PERR: $err: $!\n" if ($debug);
            @out = ( "The process did not finish within $killAfterSec seconds and had to be killed\n" );
        }
        else
        {
            print "PERR: $err: SUCCESS\n" if ($debug);
            while(<READER>)
            {
                chomp();
                push(@out, $_);
            }
            close(READER);
        }
        print "Returning from shellCmd() function\n" if ($debug);
    }
    return $err, @out;
}

1;
__END__
