/*
 * Minimal C Test Harness
 *
 * Provides TEST_BEGIN/TEST_END/RUN_TEST/TEST_SUMMARY macros and assertions.
 * Each assertion prints file:line on failure and returns from the test function.
 * TEST_SUMMARY() returns exit code suitable for CTest.
 *
 * MIT License
 */

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;
static const char *g_current_test = NULL;

#define TEST_BEGIN(name)                                    \
    do {                                                    \
        g_current_test = #name;                             \
        g_tests_run++;                                      \
        printf("  %-60s ", #name);                          \
    } while (0)

#define TEST_END()                                          \
    do {                                                    \
        g_tests_passed++;                                   \
        printf("PASS\n");                                   \
    } while (0)

#define RUN_TEST(fn)                                        \
    do {                                                    \
        fn();                                               \
    } while (0)

#define TEST_SUMMARY()                                      \
    do {                                                    \
        printf("\n%d/%d tests passed", g_tests_passed,      \
               g_tests_run);                                \
        if (g_tests_failed > 0)                             \
            printf(", %d FAILED", g_tests_failed);          \
        printf("\n");                                        \
        return g_tests_failed > 0 ? 1 : 0;                 \
    } while (0)

#define TEST_FAIL_(fmt, ...)                                \
    do {                                                    \
        printf("FAIL\n");                                   \
        printf("    %s:%d: " fmt "\n", __FILE__, __LINE__,  \
               ##__VA_ARGS__);                              \
        g_tests_failed++;                                   \
        return;                                             \
    } while (0)

#define ASSERT_TRUE(expr)                                   \
    do {                                                    \
        if (!(expr))                                        \
            TEST_FAIL_("expected true: %s", #expr);         \
    } while (0)

#define ASSERT_FALSE(expr)                                  \
    do {                                                    \
        if ((expr))                                         \
            TEST_FAIL_("expected false: %s", #expr);        \
    } while (0)

#define ASSERT_EQ(expected, actual)                         \
    do {                                                    \
        long long _e = (long long)(expected);               \
        long long _a = (long long)(actual);                 \
        if (_e != _a)                                       \
            TEST_FAIL_("expected %lld, got %lld", _e, _a);  \
    } while (0)

#define ASSERT_NEQ(expected, actual)                        \
    do {                                                    \
        long long _e = (long long)(expected);               \
        long long _a = (long long)(actual);                 \
        if (_e == _a)                                       \
            TEST_FAIL_("expected != %lld, got %lld", _e, _a); \
    } while (0)

#define ASSERT_EQ_U(expected, actual)                       \
    do {                                                    \
        unsigned long long _e = (unsigned long long)(expected); \
        unsigned long long _a = (unsigned long long)(actual);   \
        if (_e != _a)                                       \
            TEST_FAIL_("expected 0x%llX, got 0x%llX", _e, _a); \
    } while (0)

#define ASSERT_FLOAT_EQ(expected, actual, eps)              \
    do {                                                    \
        double _e = (double)(expected);                     \
        double _a = (double)(actual);                       \
        if (fabs(_e - _a) > (double)(eps))                  \
            TEST_FAIL_("expected %f, got %f (eps=%f)",      \
                       _e, _a, (double)(eps));              \
    } while (0)

#define ASSERT_NAN(val)                                     \
    do {                                                    \
        if (!isnan((double)(val)))                           \
            TEST_FAIL_("expected NAN, got %f",              \
                       (double)(val));                      \
    } while (0)

#define ASSERT_STR_EQ(expected, actual)                     \
    do {                                                    \
        const char *_e = (expected);                        \
        const char *_a = (actual);                          \
        if (strcmp(_e, _a) != 0)                            \
            TEST_FAIL_("expected \"%s\", got \"%s\"",       \
                       _e, _a);                             \
    } while (0)

#define ASSERT_MEM_EQ(expected, actual, len)                \
    do {                                                    \
        if (memcmp((expected), (actual), (len)) != 0)       \
            TEST_FAIL_("memory mismatch (%d bytes)",        \
                       (int)(len));                         \
    } while (0)

#define ASSERT_NOT_NULL(ptr)                                \
    do {                                                    \
        if ((ptr) == NULL)                                  \
            TEST_FAIL_("expected non-NULL: %s", #ptr);      \
    } while (0)

#define ASSERT_NULL(ptr)                                    \
    do {                                                    \
        if ((ptr) != NULL)                                  \
            TEST_FAIL_("expected NULL: %s", #ptr);          \
    } while (0)

#endif // TEST_HARNESS_H
