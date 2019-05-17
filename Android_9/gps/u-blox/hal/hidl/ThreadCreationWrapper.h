/*
 * NOTICE TO THE ORIGINAL WORK:
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *_________________________________________________________
 * THIS WORK WAS MODIFIED BY U-BLOX AG.
 * NOTICE TO THE DERIVATIVE WORK:
 * Copyright (C) u-blox AG, Thalwil, Switzerland
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 */

#ifndef ANDROID_HARDWARE_GNSS_THREADCREATIONWRAPPER_H
#define ANDROID_HARDWARE_GNSS_THREADCREATIONWRAPPER_H

#include <pthread.h>
#include <vector>

typedef void (*threadEntryFunc)(void *ret);

/*
 * This class facilitates createThreadCb methods in various GNSS interfaces to wrap
 * pthread_create() from libc since its function signature differs from what is required by the
 * conventional GNSS HAL. The arguments passed to pthread_create() need to be on heap and not on
 * the stack of createThreadCb.
 */
struct ThreadFuncArgs
{
  ThreadFuncArgs(void (*start)(void *), void *arg) : fptr(start), args(arg) {}

  /* pointer to the function of type void()(void*) that needs to be wrapped */
  threadEntryFunc fptr;
  /* argument for fptr to be called with */
  void *args;
};

/*
 * This method is simply a wrapper. It is required since pthread_create() requires an entry
 * function pointer of type void*()(void*) and the GNSS hal requires as input a function pointer of
 * type void()(void*).
 */
void *threadFunc(void *arg);

/*
 * This method is called by createThreadCb with a pointer to the vector that
 * holds the pointers to the thread arguments. The arg and start parameters are
 * first used to create a ThreadFuncArgs object which is then saved in the
 * listArgs parameters. The created ThreadFuncArgs object is then used to invoke
 * threadFunc() method which in-turn invokes pthread_create.
 */
pthread_t createPthread(const char *name,
                        void (*start)(void *),
                        void *arg,
                        std::vector<std::unique_ptr<ThreadFuncArgs>> *listArgs);

#endif
