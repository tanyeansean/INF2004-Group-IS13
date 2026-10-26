/**
 * @file test_util.h
 * @brief Tiny assertion harness for host unit tests (no dependencies).
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>

static int g_tests_run = 0;
static int g_tests_failed = 0;

#define CHECK(cond, msg) do {                                        \
    g_tests_run++;                                                   \
    if (!(cond)) {                                                   \
        g_tests_failed++;                                            \
        printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);  \
    } else {                                                         \
        printf("  ok:   %s\n", (msg));                               \
    }                                                                \
} while (0)

#define CHECK_EQ_INT(a, b, msg) do {                                 \
    long _a = (long)(a); long _b = (long)(b);                        \
    g_tests_run++;                                                   \
    if (_a != _b) {                                                  \
        g_tests_failed++;                                            \
        printf("  FAIL: %s  expected %ld got %ld  (%s:%d)\n",        \
               (msg), _b, _a, __FILE__, __LINE__);                   \
    } else {                                                         \
        printf("  ok:   %s\n", (msg));                               \
    }                                                                \
} while (0)

#define TEST_SUMMARY() ( \
    printf("\n%d checks, %d failed\n", g_tests_run, g_tests_failed), \
    (g_tests_failed == 0 ? 0 : 1) )

#endif /* TEST_UTIL_H */
