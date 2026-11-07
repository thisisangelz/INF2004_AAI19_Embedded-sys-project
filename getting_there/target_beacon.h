#ifndef GETTING_THERE_TARGET_BEACON_H
#define GETTING_THERE_TARGET_BEACON_H

// The beacon this robot build approaches. CMake builds one firmware per
// beacon: getting_there (AP1), robot_ap2 and robot_ap3. Any beacon may sit on
// any colour mat; the colour sensor identifies the mat from its readings.
#ifndef TARGET_BEACON_ID
#define TARGET_BEACON_ID 1
#endif
#if TARGET_BEACON_ID < 1 || TARGET_BEACON_ID > 3
#error "TARGET_BEACON_ID must be 1, 2 or 3"
#endif

#define TARGET_STR_(x) #x
#define TARGET_STR(x) TARGET_STR_(x)
// "AP1", "AP2" or "AP3" in log lines.
#define TARGET_NAME "AP" TARGET_STR(TARGET_BEACON_ID)

// Wi-Fi SSIDs set in the picow_access_point*_powerbank beacon builds.
#if TARGET_BEACON_ID == 1
#define TARGET_SSID "I AM PICO W"
#else
#define TARGET_SSID "I AM PICO W " TARGET_STR(TARGET_BEACON_ID)
#endif

#endif
