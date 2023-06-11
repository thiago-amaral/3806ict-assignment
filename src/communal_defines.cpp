#define TEST

#define VISITED -1
#define EMPTY 0
#define SUB 1
#define HOSTILE 2
#define SURVIVOR 3

#define SUB_START_X 0
#define SUB_START_Y 0

#define BOARD_H 5
#define BOARD_W 10

#define SUB_CAP 2
#define SURVIVOR_COUNT 4
#define HOSTILE_COUNT 6

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
#define BOARD_H 5
#define BOARD_W 10

#define SUB_CAP 2
#define SURVIVOR_COUNT 5
#define HOSTILE_COUNT 1
#endif

#define MAX_FUEL (BOARD_H * BOARD_W)
