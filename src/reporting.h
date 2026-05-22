#pragma once

#define REPORT_NONE 		(VERBOSITY == 0)
#define REPORT_BARE 		(VERBOSITY >= 1)
#define REPORT_STANDARD 	(VERBOSITY >= 2)
#define REPORT_EXTRA 		(VERBOSITY >= 3)
#define REPORT_VERBOSE		(VERBOSITY >= 4)
#define REPORT_ALL 			(VERBOSITY >= 5)

