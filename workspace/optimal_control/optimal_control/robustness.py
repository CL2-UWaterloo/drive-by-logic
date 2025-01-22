#!/usr/bin/python3

from casadi import *
from optimal_control.utils import *

def eventually(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lb), ge(ub, expression[-1].t))
        with_interval.append(if_else(window, expression[0], -INF))
    return logsumexp(vertcat(*with_interval), MARGIN)

def always(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lb), ge(ub, expression[-1].t))
        with_interval.append(if_else(window, expression[0], INF))
    return -logsumexp(-vertcat(*with_interval))

def eventuallyAlways(*expressions, **options):
    lbo = options["lbo"]; ubo = options["ubo"]
    lbi = options["lbi"]; ubi = options["ubi"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lbo), ge(ubo, expression[-1].t))
        lt = expression[-1].t + lbi; ut = lt + ubi
        with_interval.append(
            if_else(
                window,
                always(*expressions, lb=lt, ub=ut),
            -INF)
        )
    return logsumexp(vertcat(*with_interval), MARGIN)

def alwaysEventually(*expressions, **options):
    lbo = options["lbo"]; ubo = options["ubo"]
    lbi = options["lbi"]; ubi = options["ubi"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lbo), ge(ubo, expression[-1].t))
        lt = expression[-1].t + lbi; ut = lt + ubi
        with_interval.append(
            if_else(
                window,
                eventually(*expressions, lb=lt, ub=ut),
            INF)
        )
    return -logsumexp(-vertcat(*with_interval))

def until(**options):
    lb = options["lb"]; ub = options["ub"]
    first = options["first"]; second = options["second"]

    with_interval = []
    for expression in second:
        window = logic_and(ge(expression[-1].t, lb), ge(ub, expression[-1].t))
        with_interval_ = []
        for expression_ in first:
            window_ = ge(expression[-1].t, expression_[-1].t)
            with_interval_.append(
                if_else(
                    window_,
                    expression_[0],
                INF)
            )
        with_interval.append(
            if_else(
                window,
                fmin(expression[0], -logsumexp(-vertcat(*with_interval_))),
            -INF)
        )
    return logsumexp(vertcat(*with_interval), MARGIN)