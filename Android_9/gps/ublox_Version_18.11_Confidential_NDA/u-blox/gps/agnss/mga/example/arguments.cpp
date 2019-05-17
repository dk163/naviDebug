/******************************************************************************
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMGA
 * Purpose: Generic command line argument parser.
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Includes
#include <stdio.h>      // standard input/output definitions
#include <string.h>     // string handling
#include <stdlib.h>     // standard library

#include "arguments.h"
#include "libMga.h"

extern const char* exename;
unsigned int verbosity = 0;

bool parseArguments(int argc, char* argv[], CL_ARGUMENTS_pt clargs)
{
    // set option to unknown argument
    ARG_t option = UNKNOWN_ARG;
    ARG_t optionLast = UNKNOWN_ARG;

    // initialize command line arguments structure with default values
    memcpy(clargs, (void *)&defaultArgs, sizeof(*clargs));

    // go through all the arguments
    for (int arg = 1; arg < argc; arg++)
    {
        // check if argument selector found
        if (argv[arg][0] == '-')
        {
            //store last option
            if (option != UNKNOWN_ARG)
                optionLast = option;

            // get the option
            option = getArgAssociation(argv[arg]);

            // check for unknown argument
            if (option == UNKNOWN_ARG)
            {
                if (optionLast != UNKNOWN_ARG)
                {
                    // try to treat this as negative argument
                    setArgument(optionLast, argv[arg], clargs);
                    option = UNKNOWN_ARG;
                    optionLast = UNKNOWN_ARG;
                }
                else
                {
                    printf("Unknown argument: %s\n", argv[arg]);
                    printUsage();
                    return false;
                }
            }
            // check for help
            else if (option == HELP)
            {
                printf("Showing help as requested\n");
                printUsage();
                return false;
            }
        }
        // no argument selector
        else
        {
            // this is the argument of an option
            if (option != UNKNOWN_ARG)
            {
                // set the argument
                setArgument(option, argv[arg], clargs);
                option = UNKNOWN_ARG;
                optionLast = UNKNOWN_ARG;
            }
            // standalone argument
            else
            {
                printf("Standalone argument %s\n\n", argv[arg]);
            }
        }
    }

    return true;
}

ARG_t getArgAssociation(char* arg)
{
    unsigned int idx = 0;
    while (idx < sizeof(knownArgs) / sizeof(*knownArgs))
    {
        if (strcmp(arg, knownArgs[idx].argstr) == 0)
        {
            return knownArgs[idx].argass;
        }
        idx++;
    }
    return UNKNOWN_ARG;
}

// function to set the argument
bool setArgument(ARG_t option, char *value, CL_ARGUMENTS_pt clargs)
{
    // switch depending on option
    switch (option)
    {
    case TOKEN:             clargs->token = value;              break;
    case TEST:              clargs->test = value;               break;
    case BAUDRATE:          clargs->baudrate = atol(value);     break;
    case PORT:              clargs->port = value;               break;
    case SERVER1:           clargs->server1 = value;            break;
    case SERVER2:           clargs->server2 = value;            break;
    case GNSS:              clargs->gnss = value;               break;
    case TIMEOUT:           clargs->timeout = atol(value);      break;
    case DATATYPE:          clargs->dataType = value;           break;
    case FORMAT:            clargs->format = value;             break;
    case ALMANAC:           clargs->alm = value;                break;
    case RETRY:             clargs->retry = atol(value);        break;
    case FLOW:              clargs->flow = value;               break;
    case USEPOSITION:       clargs->usePosition = atol(value);  break;
    case ACCURACY:          clargs->posAccuracy = atol(value);  break;
    case ALTITUDE:          clargs->posAltitude = atof(value);  break;
    case LATITUDE:          clargs->posLatitude = atof(value);  break;
    case LONGITUDE:         clargs->posLongitude = atof(value); break;
    case USELATENCY:        clargs->useLatency = atol(value);   break;
    case LATENCY:           clargs->latency = atol(value);      break;
    case USETIMEACCURACY:   clargs->useTimeAccuracy = atol(value); break;
    case TIMEACCURACY:      clargs->timeAccuracy = atof(value); break;
    case PERIOD:            clargs->period = atol(value);       break;
    case RESOLUTION:        clargs->resolution = atol(value);   break;
    case DAYS:              clargs->days = atol(value);         break;
    case LEGACYHOSTDUR:     clargs->legacyServerDuration = atoi(value); break;
    case VERBOSITY:
        clargs->verbosity = atol(value);
        verbosity = (unsigned int)atol(value);
        break;
    case SERVERVERIFY:      clargs->serverVerify = atoi(value);       break;
    default:
        printUsage();
        break;
    }

    return true;
}

void printUsage(void)
{
    printf("\n");
    printf("NAME\n");
    printf("    libMGA Example (%s)\n", exename);
    printf("    using libMGA v%s\n", mgaGetVersion());
    printf("\n");

    // print the description
    printf("DESCRIPTION\n");
    printf("    Performs aiding functionality for u-blox receivers (u-blox 5 and later).\n");
    printf("    The tool connects to the u-blox MGA server(s), downloads the required data\n");
    printf("    and send this information to the u-blox GNSS receiver. It can be used for\n");
    printf("    legacy data (UBX-AID-*) or MGA data (UBX-MGA-*).\n");
    printf("    The example is used to show the usage of the libMGA and runs under Windows\n");
    printf("    and Linux.\n");
    printf("\n");

    // print the exit status
    printf("EXIT STATUS\n");
    printf("    %d on success\n", EXIT_SUCCESS);
    printf("    %d on failure\n", EXIT_FAILURE);
    printf("\n");

    // print the options and the known default values
    printf("OPTIONS:\n");

    unsigned int idx = 0;
    while (idx < sizeof(knownArgs) / sizeof(*knownArgs))
    {
        printf("    %-10s %-30s\n", knownArgs[idx].argstr, knownArgs[idx].argdesc);

        switch (knownArgs[idx].argass)
        {
        case BAUDRATE:
            printf("                 (default = %d)\n", defaultArgs.baudrate);
            break;
        case PORT:
            printf("               \\\\.\\COMy   - serial (RS232) port y (Windows)\n");
            printf("               /dev/ttySy - serial (RS232) port y (Linux)\n");
            printf("                 (default = '%s')\n", defaultArgs.port);
            break;
        case SERVER1:
            printf("                 (default = '%s')\n", defaultArgs.server1);
            break;
        case SERVER2:
            printf("                 (default = '%s')\n", defaultArgs.server2);
            break;
        case GNSS:
            printf("                 GNSS systems to request data for. Possible systems:\n");
            printf("                   gps = GPS, glo = GLONASS, gal = Galileo\n");
            printf("                   bds = BeiDou, qzss = QZSS\n");
            printf("                 (default = '%s')\n", defaultArgs.gnss);
            break;
        case TEST:
            printf("                 Possible tests:\n");
            printf("                   online      = MGA and legacy online test\n");
            printf("                   offline     = MGA offline host based test\n");
            printf("                   flash       = MGA offline flash based test\n");
            printf("                   legacyhost  = Legacy offline host based test (GPS only)\n");
            printf("                   legacyflash = Legacy offline flash based test (GPS only)\n");
            printf("                 (default = '%s')\n", defaultArgs.test);
            break;
        case TIMEOUT:
            printf("                 (default = %d)\n", defaultArgs.timeout);
            break;
        case DATATYPE:
            printf("                 Possible data types:\n");
            printf("                   eph = Ephemeris\n");
            printf("                   alm = Almanac\n");
            printf("                   aux = Auxilary\n");
            printf("                   pos = Position\n");
            printf("                 (default = '%s')\n", defaultArgs.dataType);
            break;
        case FORMAT:
            printf("                 Possible data formats:\n");
            printf("                   mga = Multiple GNSS AssistNow (MGA) format\n");
            printf("                   aid = Legacy AssistNow format (GPS only)\n");
            printf("                 (default = '%s')\n", defaultArgs.format);
            break;
        case ALMANAC:
            printf("                 GNSS systems to request Almanac data for. Possible systems:\n");
            printf("                   gps = GPS, glo = GLONASS, gal = Galileo\n");
            printf("                   bds = BeiDou, qzss = QZSS\n");
            printf("                 (default = '%s')\n", defaultArgs.gnss);
            break;
        case RETRY:
            printf("                 (default = %d)\n", defaultArgs.retry);
            break;
        case FLOW:
            printf("                 Possible flow types:\n");
            printf("                   none   = No flow control\n");
            printf("                   simple = Simple flow control\n");
            printf("                   smart  = Smart flow control\n");
            printf("                 (default = '%s')\n", defaultArgs.flow);
            break;
        case USEPOSITION:
            printf("                 (default = %d)\n", defaultArgs.usePosition);
            break;
        case ACCURACY:
            printf("                 (default = %f)\n", defaultArgs.posAccuracy);
            break;
        case ALTITUDE:
            printf("                 (default = %f)\n", defaultArgs.posAltitude);
            break;
        case LATITUDE:
            printf("                 (default = %f)\n", defaultArgs.posLatitude);
            break;
        case LONGITUDE:
            printf("                 (default = %f)\n", defaultArgs.posLongitude);
            break;
        case USELATENCY:
            printf("                 (default = %d)\n", defaultArgs.useLatency);
            break;
        case LATENCY:
            printf("                 (default = %f)\n", defaultArgs.latency);
            break;
        case USETIMEACCURACY:
            printf("                 (default = %d)\n", defaultArgs.useTimeAccuracy);
            break;
        case TIMEACCURACY:
            printf("                 (default = %f)\n", defaultArgs.timeAccuracy);
            break;
        case VERBOSITY:
            printf("                 (default = %d)\n", defaultArgs.verbosity);
            break;
        case PERIOD:
            printf("                 Possible periods: 1 ... 5\n");
            printf("                 (default = %d)\n", defaultArgs.verbosity);
            break;
        case RESOLUTION:
            printf("                 Possible resolution\n");
            printf("                   1 = Every day\n");
            printf("                   2 = Every other day\n");
            printf("                   3 = Every third day\n");
            printf("                 (default = %d)\n", defaultArgs.resolution);
            break;
        case DAYS:
            printf("                 Possible days for aid format: 1, 2, 3, 5, 7, 10, 14\n");
            printf("                 Possible days for mga format: 1 - 35\n");
            printf("                 (default for mga format = %d)\n", defaultArgs.days);
            printf("                 (default for aid format = 14)\n");
            break;
        case LEGACYHOSTDUR:
            printf("                 Duration to run legacy aiding host server (in seconds)\n");
            printf("                 (default = %d)\n", defaultArgs.legacyServerDuration);
            break;
        case SERVERVERIFY:
            printf("                 Server certificate verification (1 or 0)\n");
            printf("                 (default = %d)\n", defaultArgs.serverVerify);
            break;
        default:
            break;
        }

        printf("\n");

        // go to next known argument
        idx++;
    }

    // print the examples
    printf("\n");
    printf("EXAMPLES\n");
    printf("    test MGA online with GPS data on COM1 at 9600 baud\n");
    printf("    %s -token <token> -test online -p \\\\.\\COM1 -b 9600 -gnss gps\n", exename);
    printf("\n");

    // print the author information
    printf("AUTHOR\n");
    printf("    u-blox software development team (www.u-blox.com)\n");

}

void printConfiguration(CL_ARGUMENTS_pt clargs)
{
    printf("CONFIGURATION\n");
    printf("  Interface configuration\n");
    printf("    %-17s: %-30s\n", "Port", clargs->port);
    printf("    %-17s: %-30d\n", "Baud rate", clargs->baudrate);
    printf("  Server configuration\n");
    printf("    %-17s: %-30s\n", "Server1", clargs->server1);
    printf("    %-17s: %-30s\n", "Server2", clargs->server2);
    printf("    %-17s: %-30s\n", "Token", clargs->token);
    printf("    %-17s: %-30s\n", "GNSS", clargs->gnss);
    printf("    %-17s: %-30s\n", "Format", clargs->format);
    printf("    %-17s: %-30s\n", "Data", clargs->dataType);
    printf("    %-17s: %-30s\n", "Almanac", clargs->alm);
    printf("  Test configuration\n");
    printf("    %-17s: %-30d\n", "Verbosity", clargs->verbosity);
    printf("    %-17s: %-30s\n", "Test", clargs->test);
    printf("    %-17s: %-30d\n", "Timeout", clargs->timeout);
    printf("    %-17s: %-30d\n", "Retry", clargs->retry);
    printf("    %-17s: %-30s\n", "Flow control", clargs->flow);

    // check if online test
    if (strcmp(clargs->test, "online") == 0)
    {
        printf("    %-17s: %-30d\n", "Use position", clargs->usePosition);
        if (clargs->usePosition == 1)
        {
            printf("      %-17s: %-30f\n", "Accuracy", clargs->posAccuracy);
            printf("      %-17s: %-30f\n", "Altitude", clargs->posAltitude);
            printf("      %-17s: %-30f\n", "Latitude", clargs->posLatitude);
            printf("      %-17s: %-30f\n", "Longitude", clargs->posLongitude);
        }

        printf("    %-17s: %-30d\n", "Use latency", clargs->useLatency);
        if (clargs->useLatency == 1)
            printf("      %-17s: %-30f\n", "Latency", clargs->latency);

        printf("    %-17s: %-30d\n", "Use time accuracy", clargs->useTimeAccuracy);
        if (clargs->useTimeAccuracy)
            printf("      %-17s: %-30f\n", "Time accuracy", clargs->timeAccuracy);
    }

    // check if offline test
    if ((strcmp(clargs->test, "offline") == 0) ||
        (strcmp(clargs->test, "flash") == 0))
    {
        printf("    %-17s: %-30d\n", "Period", clargs->period);
        printf("    %-17s: %-30d\n", "Resolution", clargs->resolution);
        printf("    %-17s: %-30d\n", "Days", clargs->days);
    }

    // check if legacy offline host based test
    if (strcmp(clargs->test, "legacyhost") == 0)
    {
        printf("    %-17s: %-30d\n", "Days", clargs->days);
        printf("    %-17s: %-30d\n", "Legacy host duration", clargs->legacyServerDuration);
    }

    // check if legacy offline flash test
    if (strcmp(clargs->test, "legacyflash") == 0)
    {
        printf("    %-17s: %-30d\n", "Days", clargs->days);
    }

    printf("    %-17s: %-30d\n", "Verify server certificate", clargs->serverVerify);


    printf("\n");
}
