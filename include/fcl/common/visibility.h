/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _FCL_VISIBLE_HH_
#define _FCL_VISIBLE_HH_

/** \def FCL_VISIBLE
 * Use to represent "symbol visible" if supported
 */

/** \def FCL_HIDDEN
 * Use to represent "symbol hidden" if supported
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define FCL_VISIBLE __attribute__ ((dllexport))
    #else
      #define FCL_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define FCL_VISIBLE __attribute__ ((dllimport))
    #else
      #define FCL_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define FCL_HIDDEN
#else
  #if __GNUC__ >= 4
    #define FCL_VISIBLE __attribute__ ((visibility ("default")))
    #define FCL_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define FCL_VISIBLE
    #define FCL_HIDDEN
  #endif
#endif

#endif /* FCL_VISIBLE_HH */
