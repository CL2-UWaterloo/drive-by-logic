#!/usr/bin/python3

from optimal_control.utils import *

"""
Relevant: Smooth Operator. Pant et Al
https://github.com/casadi/casadi/issues/2893
"""

def smooth_max(signals : ListVector):
    return logsumexp(vertcat(*signals), MARGIN)

def smooth_min(signals : ListVector):
    return -logsumexp(-vertcat(*signals), MARGIN)

def process_options(signalslen : int, **options):
    t0 = 0.0 # What is the first signals's time stamp
    if "t0" in options.keys():
        t0 : float = options["t0"]
    
    ta = t0
    if "lb" in options.keys():
        ta = options["lb"] # When does the signal start getting evaluated

    tb = None # if none all signals from ta are evaluated
    if "ub" in options.keys():
        tb = options["ub"] # What is the last time stamp to be evaluated.

    dt = options["dt"]

    # tai is the index where the signal starts
    tai = None
    if ta != t0:
        for i in range(signalslen):
            ti = t0 + i*dt
            if floor(ti - ta) == 0:
                tai = i
                break
    else:
        tai = 0

    # tbi is the index where the signal ends
    tbi = None
    if tb != None:
        for i in range(signalslen):
            ti = t0 + i*dt
            if floor(ti - tb) == 0:
                tbi = i
                break
    else:
        tbi = signalslen-1
    
    return t0, ta, tb, dt, tai, tbi

def always(signals : ListVector, **options):
    """
    Formula: G_{[ta, tb]} (signals) -> Always uphold the signals for the time interval
    Options:
        Mandatory: dt
        Optional: t0, ta, tb. Default: 0.0, 0.0, NaN
    """
    t0, ta, tb, dt, tai, tbi = process_options(len(signals), **options)
    return smooth_min(signals[tai:tbi+1])

def eventually(signals : ListVector, **options):
    """
    Formula: F_{[ta, tb]} (signals) -> Eventually uphold the signals in the time interval
    Options:
        Mandatory: dt
        Options: t0, ta, tb. Default: 0.0, 0.0, NaN
    """
    t0, ta, tb, dt, tai, tbi = process_options(len(signals), **options)
    return smooth_max(signals[tai:tbi+1])

def until(signals1 : ListVector, signals2: ListVector, **options):
    """
    signals1 until signals2
    """
    t0, ta, tb, dt, tai, tbi = process_options(len(signals1), **options)
    outer_max_signals = []
    for i in range(tai, tbi+1):
        outer_max_signals.append(
            smooth_min([signals2[i], smooth_min(signals1[0:i+1])])
        )
    return smooth_max(outer_max_signals)