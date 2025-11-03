/*
 * tasks.h
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_TASKS_H_
#define INCLUDE_TASKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASKS 8

typedef struct task {
    void (*run)(void *);
    void *arg;
} task_t;

extern void ClearTasks();
extern int32_t SubmitTask(void (*run)(void *), void *arg);
extern void CheckForTask();

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_TASKS_H_ */
