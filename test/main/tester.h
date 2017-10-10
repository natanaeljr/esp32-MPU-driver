#ifndef _TESTER_H_
#define _TESTER_H_

#include "esp_err.h"
#include "esp_log.h"



/* Conventional naming */
#define ASSERT_ESP_OK(x)                        TEST_ASSERT_ESP_OK(x)
#define ASSERT_ESP_ERR(x)                       TEST_ASSERT_ESP_ERR(x)
#define ASSERT_TRUE(x)                          TEST_ASSERT_TRUE(x)
#define ASSERT_FALSE(x)                         TEST_ASSERT_FALSE(x)
#define ASSERT_EQUAL_INT(x, y)                  TEST_ASSERT_EQUAL_INT(x, y)
#define ASSERT_NOTEQUAL_INT(x, y)               TEST_ASSERT_NOTEQUAL_INT(x, y)



/* Real test macros */
#define TEST_ASSERT_ESP_OK(x) do {\
    esp_err_t err = (esp_err_t)x;\
    if(err) {\
        TEST_LOGFAIL(#x, "ESP_OK", "0x%X", err);\
    } else {\
        TEST_LOGPASS(#x, "%X", err);\
    }}while(0);


#define TEST_ASSERT_ESP_ERR(x) do {\
    esp_err_t err = (esp_err_t)x;\
    if(!err) {\
        TEST_LOGFAIL(#x, "ERROR", "0x%X", err);\
    } else {\
        TEST_LOGPASS(#x, "0x%x", err);\
    }}while(0);


#define TEST_ASSERT_TRUE(x) do {\
    int cond = (int)x;\
    if(!cond) {\
        TEST_LOGFAIL(#x, "TRUE", "FALSE %d", cond);\
    } else {\
        TEST_LOGPASS(#x, "%d", cond);\
    }}while(0);


#define TEST_ASSERT_FALSE(x) do {\
    int cond = (int)x;\
    if(cond) {\
        TEST_LOGFAIL(#x, "FALSE", "TRUE %d", cond);\
    } else {\
        TEST_LOGPASS(#x, "%d", cond);\
    }}while(0);


#define TEST_ASSERT_EQUAL_INT(x, y) do {\
    int actual = (int)x;\
    int expected = (int)y;\
    if(actual == expected) {\
        TEST_LOGPASS(#x, "%d", actual);\
    } else {\
        TEST_LOGFAIL(#x, "%d", "%d", expected, actual);\
    }}while(0);


#define TEST_ASSERT_NOTEQUAL_INT(x, y) do {\
    int actual = (int)x;\
    int expected = (int)y;\
    if(actual != expected) {\
        TEST_LOGPASS(#x, "%d", actual);\
    } else {\
        TEST_LOGFAIL(#x, "NOT(%d)", "%d", expected, actual);\
    }}while(0);






/* Logging Macros */
#define TEST_LOGFAIL(expr, expected, actual, ... ) \
printf(LOG_COLOR_E "[%s] @ %d '" expr "' -> FAIL! expected: " expected ", actual: " actual LOG_RESET_COLOR "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#ifdef CONFIG_TESTER_LOG_ASSERTION_PASS
#define TEST_LOGPASS(expr, actual, ... ) \
printf("[%s] @ %d '" expr "' -> " actual " PASS\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define TEST_LOGPASS(expr, actual, ... ) (void)0
#endif









#endif /* end of include guard: _TESTER_H_ */