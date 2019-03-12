#ifndef FCL_EXPECT_THROWS_MESSAGE_H
#define FCL_EXPECT_THROWS_MESSAGE_H

#include <regex>
#include <string>


#define FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                         must_throw, fatal_failure) \
try { \
  expression; \
  if (must_throw) { \
    if (fatal_failure) { \
      GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
    } else { \
      GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception);\
    } \
  } \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  if (fatal_failure) { \
    ASSERT_PRED2(matcher, err.what(), regexp); \
  } else { \
    EXPECT_PRED2(matcher, err.what(), regexp); \
  } \
}

#define FCL_EXPECT_THROWS_MESSAGE(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, false /*non-fatal*/)

#define FCL_ASSERT_THROWS_MESSAGE(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   true /*must_throw*/, true /*fatal*/)

#ifdef NDEBUG
// Throwing the expected message is optional in this case.

#define FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   false /*optional*/, false /*non-fatal*/)

#define FCL_ASSERT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                   false /*optional*/, true /*fatal*/)

#else  //
// Throwing the expected message is required in this case.

#define FCL_EXPECT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

#define FCL_ASSERT_THROWS_MESSAGE_IF_DEBUG(expression, exception, regexp) \
  FCL_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

#endif

#endif  // FCL_EXPECT_THROWS_MESSAGE_H
