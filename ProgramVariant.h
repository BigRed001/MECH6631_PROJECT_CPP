// Modes 
#define MODE_OFFENSE 1
#define MODE_DEFENSE 2

// Robot color profiles
// BR: blue front / red rear, legacy profile
// GR: green front / red rear, robot A by default
// OB: orange front / blue rear, robot B by default
// AUTO: unknown robot profile, use ID dance to identify our robot
#define PROFILE_AUTO 0
#define PROFILE_BR   1
#define PROFILE_GR   2
#define PROFILE_OB   3


#define PROGRAM_DETECT_BR_PROFILE 0
#define ENABLE_DIAGNOSTICS 1
#define DIAG_CSV_PATH "run_diagnostics.csv"

// ============================================================
// TUNE FOR EACH SUBMISSION FOLDER
// ============================================================
#define PROGRAM_MODE MODE_OFFENSE
#define PROGRAM_MY_PROFILE PROFILE_AUTO
#define PROGRAM_USE_ID_DANCE 1



