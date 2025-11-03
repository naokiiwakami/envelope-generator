/*
 * tasks.cpp
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#include <stdint.h>

#include "main.h"

#include "analog3/tasks.h"

task_t pending_tasks[MAX_TASKS];
volatile uint8_t task_first;
volatile uint8_t task_last;
volatile uint8_t tasks_overflow;

void ClearTasks() {
  task_first = 0;
  task_last = 0;
  tasks_overflow = 0;
}

int32_t SubmitTask(void (*run)(void *), void *arg) {
  if (tasks_overflow) {
    return -1;
  }
  pending_tasks[task_last].run = run;
  pending_tasks[task_last].arg = arg;
  task_last = (task_last + 1) % MAX_TASKS;
  if (task_last == task_first) {
    tasks_overflow = 1;
  }
  return 0;
}

void CheckForTask() {
  if (task_last == task_first && !tasks_overflow) {
    return;
  }
  task_t task = pending_tasks[task_first]; // clone the task
  __disable_irq();  // enter critical section
  task_first = (task_first + 1) % MAX_TASKS;
  tasks_overflow = 0;
  __enable_irq();  // exit critical section
  task.run(task.arg);
#if 0
  __disable_irq();  // enter critical section
  if (task_last == task_first && !tasks_overflow) {
    // nothing to pick up
    __enable_irq();  // exit critical section
    return;
  }
  task_t task = pending_tasks[task_first];
  task_first = (task_first + 1) % MAX_TASKS;
  tasks_overflow = 0;
  __enable_irq();  // exit critical section
  task.run(task.arg);
#endif
}
