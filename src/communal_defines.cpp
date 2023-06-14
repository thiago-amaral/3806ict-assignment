// #define TEST

#define VISITED -1
#define EMPTY 0
#define SUB 1
#define HOSTILE 2
#define SURVIVOR 3

#define SUB_START_X 0
#define SUB_START_Y 0

#define BOARD_H 15
#define BOARD_W 15

#define SUB_CAP 2
#define SURVIVOR_COUNT 5
#define HOSTILE_COUNT 8

#define HOSTILE_DETECTION_RANGE 2
#define SURVIVOR_DETECTION_RANGE 1

#ifdef TEST
// undef everything
#undef BOARD_H
#undef BOARD_W

#undef SUB_CAP
#undef SURVIVOR_COUNT
#undef HOSTILE_COUNT

// redef everything
#define BOARD_H 13
#define BOARD_W 13

#define SUB_CAP 2
#define SURVIVOR_COUNT 3
#define HOSTILE_COUNT 0
#endif

#define MAX_FUEL (BOARD_H * BOARD_W)
