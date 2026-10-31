"""Optional CPU selection for repeatable mapping-process validation."""
import os


def with_cpu_affinity(command, cpu):
    if cpu is None:
        return list(command)
    if cpu not in os.sched_getaffinity(0):
        raise ValueError('Requested mapping CPU is not allowed: ' + str(cpu))
    return ['taskset', '--cpu-list', str(cpu), *command]
